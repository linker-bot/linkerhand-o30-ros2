/* URDF 3D 查看器 (通用灵巧手) — three.js (本地) + 轻量 URDF 解析器
 * 数据流: /api/joints (关节映射+弧度限位, 未就绪时轮询重试) + /api/urdf (XML, mesh 已改写为 /api/mesh/...)
 * 姿态源: window.handState {names, values} (由 index.html 的滑动条维护)
 */
import * as THREE from 'three';
import { OrbitControls } from '/lib/three/examples/jsm/controls/OrbitControls.js';
import { STLLoader } from '/lib/three/examples/jsm/loaders/STLLoader.js';

const container = document.getElementById('urdf-view');
const statusEl = document.getElementById('urdf-status');

let renderer = null, scene = null, camera = null, controls = null;
let joints = null;    // { urdf关节名: { obj, axis } }
let jointMeta = [];   // [{state, urdf, min_rad, max_rad}]

function setStatus(msg, cls = '') {
  if (statusEl) { statusEl.textContent = msg; statusEl.className = cls; }
}

/* ---------- 基础数学工具 ---------- */
function parseVec3(s, def = '0 0 0') {
  const p = String(s || def).trim().split(/[\s,]+/).filter(Boolean).map(Number);
  while (p.length < 3) p.push(0);
  return new THREE.Vector3(p[0] || 0, p[1] || 0, p[2] || 0);
}
function rpyToQuat(roll, pitch, yaw) {
  const X = new THREE.Vector3(1, 0, 0), Y = new THREE.Vector3(0, 1, 0),
        Z = new THREE.Vector3(0, 0, 1);
  const q = (v, a) => new THREE.Quaternion().setFromAxisAngle(v, a);
  // URDF 约定: R = Rz(yaw) * Ry(pitch) * Rx(roll)
  return q(Z, yaw).multiply(q(Y, pitch)).multiply(q(X, roll));
}
function applyPose(obj, el) {
  if (!el) return;
  obj.position.copy(parseVec3(el.getAttribute('xyz')));
  const r = (el.getAttribute('rpy') || '0 0 0').trim().split(/\s+/).map(Number);
  obj.quaternion.copy(rpyToQuat(r[0] || 0, r[1] || 0, r[2] || 0));
}

/* ---------- URDF → THREE 场景 ---------- */
async function buildUrdf(xmlText) {
  const doc = new DOMParser().parseFromString(xmlText, 'application/xml');
  const stl = new STLLoader();
  const errEl = doc.querySelector('parsererror');
  if (errEl) throw new Error('URDF XML 解析失败');

  const defaultMat = new THREE.MeshStandardMaterial({
    color: 0xcfd4dd, roughness: 0.55, metalness: 0.25, side: THREE.DoubleSide });
  const materials = {};
  doc.querySelectorAll('material').forEach(m => {
    const name = m.getAttribute('name');
    if (!name) return;
    const c = m.querySelector('color');
    if (c) {
      const r = (c.getAttribute('rgba') || '1 1 1 1').split(/\s+/).map(Number);
      materials[name] = new THREE.MeshStandardMaterial({
        color: new THREE.Color(r[0] || 1, r[1] || 1, r[2] || 1),
        roughness: 0.55, metalness: 0.25, side: THREE.DoubleSide });
    } else {
      materials[name] = defaultMat;
    }
  });

  const links = {};
  const meshTasks = [];
  doc.querySelectorAll('link').forEach(l => {
    const name = l.getAttribute('name');
    const group = new THREE.Group();
    l.querySelectorAll('visual').forEach(v => {
      const geoEl = v.querySelector('geometry');
      if (!geoEl) return;
      const matRef = v.querySelector('material');
      const mat = (matRef && materials[matRef.getAttribute('name')]) || defaultMat;
      const meshEl = geoEl.querySelector('mesh');
      const boxEl = geoEl.querySelector('box');
      const cylEl = geoEl.querySelector('cylinder');
      const sphEl = geoEl.querySelector('sphere');
      let meshObj = null;

      if (boxEl || cylEl || sphEl) {
        let geometry = null;
        if (boxEl) geometry = new THREE.BoxGeometry(...parseVec3(boxEl.getAttribute('size'), '1 1 1').toArray());
        else if (cylEl) {
          const rad = parseFloat(cylEl.getAttribute('radius')) || 0.01;
          const len = parseFloat(cylEl.getAttribute('length')) || 0.01;
          geometry = new THREE.CylinderGeometry(rad, rad, len);
        } else {
          const rad = parseFloat(sphEl.getAttribute('radius')) || 0.01;
          geometry = new THREE.SphereGeometry(rad, 24, 16);
        }
        meshObj = new THREE.Mesh(geometry, mat);
      } else if (meshEl) {
        const fname = meshEl.getAttribute('filename');
        meshObj = new THREE.Mesh(undefined, mat);
        meshTasks.push((async () => {
          try {
            const buf = await (await fetch(fname)).arrayBuffer();
            const g = stl.parse(buf);   // STLLoader.parse 为同步方法 (r158)
            g.computeVertexNormals();
            meshObj.geometry = g;
          } catch (e) {
            console.warn('[URDF] mesh 加载失败:', fname, e);
          }
        })());
      } else {
        return;
      }
      if (meshObj) {
        applyPose(meshObj, v.querySelector('origin'));
        if (meshEl) {
          const sc = meshEl.getAttribute('scale');
          if (sc) meshObj.scale.copy(parseVec3(sc, '1 1 1'));
        }
        group.add(meshObj);
      }
    });
    links[name] = group;
  });

  const jointsOut = {};
  const childSet = new Set();
  doc.querySelectorAll('joint').forEach(j => {
    const type = j.getAttribute('type') || 'fixed';
    const pEl = j.querySelector('parent'), cEl = j.querySelector('child');
    if (!pEl || !cEl) return;
    const parent = pEl.getAttribute('link'), child = cEl.getAttribute('link');
    childSet.add(child);
    if (!parent || !child || !links[parent] || !links[child]) return;
    const axisEl = j.querySelector('axis');
    const axis = axisEl ? parseVec3(axisEl.getAttribute('xyz'), '1 0 0')
                         : new THREE.Vector3(1, 0, 0);
    if (type === 'fixed' || type === 'mimic') {
      applyPose(links[child], j.querySelector('origin'));
      links[parent].add(links[child]);
    } else {
      const jobj = new THREE.Group();
      applyPose(jobj, j.querySelector('origin'));
      jobj.add(links[child]);
      links[parent].add(jobj);
      // 关节原点 rpy 是"零位置静态偏转"(URDF 注释: zero offsets baked into origins),
      // 之前 animate 用 setFromAxisAngle 直接覆盖会把它丢掉(拇指大偏转 → 姿态错)。
      // 此处保存, 渲染时把角度叠加在该静态偏转之上, 与标准 URDF 渲染一致。
      jobj.userData.restQuat = jobj.quaternion.clone();
      jointsOut[j.getAttribute('name')] = { obj: jobj, axis, restQuat: jobj.userData.restQuat };
    }
  });

  await Promise.all(meshTasks);
  const rootName = Object.keys(links).find(n => !childSet.has(n))
    || Object.keys(links)[0];
  return { root: links[rootName], joints: jointsOut,
           linkCount: Object.keys(links).length };
}

/* ---------- 渲染循环: 每帧按滑动条值(=真机值)设置关节角度 ---------- */
const _axis = new THREE.Vector3();
const _jointQ = new THREE.Quaternion();
function animate() {
  requestAnimationFrame(animate);
  if (joints && jointMeta.length && window.handState) {
    const st = window.handState;
    for (const jm of jointMeta) {
      const j = joints[jm.urdf];
      if (!j) continue;
      const i = st.names.indexOf(jm.state);
      if (i < 0) continue;
      let v = Number(st.values[i]);
      if (!Number.isFinite(v)) v = 0;
      v = Math.max(0, Math.min(255, v));
      const angle = jm.min_rad + (v / 255) * (jm.max_rad - jm.min_rad);
      _axis.copy(j.axis);
      if (_axis.lengthSq() < 1e-12) _axis.set(1, 0, 0);
      _axis.normalize();
      // 角度必须叠加在关节原点的静态偏转(restQuat)之上, 而不是覆盖它。
      if (j.restQuat) {
        _jointQ.setFromAxisAngle(_axis, angle);
        j.obj.quaternion.copy(j.restQuat).multiply(_jointQ);
      } else {
        j.obj.quaternion.setFromAxisAngle(_axis, angle);
      }
    }
  }
  if (controls) controls.update();
  if (renderer && scene && camera) renderer.render(scene, camera);
}

function resize() {
  if (!renderer || !container || !container.clientWidth) return;
  const w = container.clientWidth, h = container.clientHeight;
  renderer.setSize(w, h);   // updateStyle=true: canvas CSS 尺寸与容器一致
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}

/* 显示器缩放 (dpr) 变化时重设渲染像素比并重新布局:
   系统缩放调整 / 浏览器缩放 / 跨显示器移动窗口后 3D 视图保持清晰 (1s 轮询兜底) */
let lastDpr = window.devicePixelRatio;
setInterval(() => {
  const d = window.devicePixelRatio;
  if (d === lastDpr) return;
  lastDpr = d;
  if (renderer) {
    renderer.setPixelRatio(Math.min(d, 3));
    resize();
  }
}, 1000);

/* 计算网格的体积加权质心 (比包围盒中心更接近视觉重心, 用于相机取景居中) */
function computeCOM(root, fallback) {
  const acc = new THREE.Vector3();
  const tmpBox = new THREE.Box3();
  const tmpC = new THREE.Vector3();
  let vol = 0;
  root.traverse(o => {
    if (!o.isMesh || !o.geometry) return;
    o.geometry.computeBoundingBox();
    const b = o.geometry.boundingBox;
    const v = (b.max.x - b.min.x) * (b.max.y - b.min.y) * (b.max.z - b.min.z);
    if (v <= 1e-12) return;
    tmpBox.copy(b).applyMatrix4(o.matrixWorld);
    tmpBox.getCenter(tmpC);
    acc.addScaledVector(tmpC, v);
    vol += v;
  });
  if (vol > 1e-12) return acc.multiplyScalar(1 / vol);
  return fallback;
}

/* ---------- 启动 ---------- */
async function init() {
  try {
    // WebGL 前置检查 (部分机器 Edge/驱动下会直接黑屏, 这里给出明确提示)
    const probe = document.createElement('canvas');
    if (!(probe.getContext('webgl2') || probe.getContext('webgl'))) {
      throw new Error('WebGL 不可用: 请在 Edge 设置→系统 开启「可用时启用硬件加速」, 更新显卡驱动, 或换用最新版 Chrome');
    }
    renderer = new THREE.WebGLRenderer({ antialias: true,
      powerPreference: 'high-performance' });
    renderer.setClearColor(0x10141f, 1);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 3));   // 上限 3: 高倍系统缩放下保持清晰
    container.appendChild(renderer.domElement);
    renderer.domElement.addEventListener('webglcontextlost', (ev) => {
      ev.preventDefault();
      setStatus('WebGL 上下文丢失 (GPU/驱动异常), 请刷新页面', 'err');
    });

    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera(45, 1, 0.001, 100);
    camera.position.set(0.35, 0.25, 0.35);
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.12;

    scene.add(new THREE.HemisphereLight(0xbfd4ff, 0x1a2030, 1.2));
    const key = new THREE.DirectionalLight(0xffffff, 1.6);
    key.position.set(0.5, 1.0, 0.75);
    scene.add(key);
    const fill = new THREE.DirectionalLight(0x88aaff, 0.6);
    fill.position.set(-0.6, 0.3, -0.5);
    scene.add(fill);

    // 关节映射 (状态名 → URDF 关节名 + 弧度限位)
    // 状态消息 / URDF 未就绪时服务端返回 ok=false: retryable=true 则轮询重试, 否则直接报错
    let meta = null;
    for (let i = 0; i < 90; i++) {
      const m = await (await fetch('/api/joints')).json();
      if (m && m.ok) { meta = m; break; }
      if (m && m.ok === false && m.retryable === false) {
        throw new Error(m.error || '/api/joints 失败');
      }
      setStatus((m && m.error) || '等待关节映射 (状态消息 / URDF 未就绪)…');
      await new Promise(r => setTimeout(r, 1000));
    }
    if (!meta) throw new Error('等待关节映射超时: 未收到状态消息 /cb_*_hand_state 或 URDF 缺失');
    jointMeta = meta.joints;

    // URDF 文本 (mesh 引用已被服务端改写为 /api/mesh/...)
    const text = await (await fetch('/api/urdf')).text();
    const built = await buildUrdf(text);
    joints = built.joints;
    scene.add(built.root);
    built.root.updateMatrixWorld(true);

    // —— 姿态修正: hand_base_link 坐标系刚性对齐 —— 安装面(局部 z=0 面)严格平行并平贴 y=0 网格 + 手指竖直朝上 + 薄面(拇指侧)水平朝向相机方位 ——
    // 说明: 竖直轴取 base 自身主轴而非指尖方向 (指尖受指链关节偏移, 与 base z 轴相差约 9°, 直接用会整体倾斜);
    //       该姿态下 hand_base_link 安装面即整只手最低面, 严格水平贴地 (左右手均适用)。
    {
      const up = new THREE.Vector3(0, 1, 0);
      const camDirH = new THREE.Vector3(0.62, 0, 0.62).normalize(); // 相机水平方位

      // 1) 指尖方向: 距手腕(根原点)最远的顶点
      let tip = null, best = -1;
      built.root.traverse(o => {
        if (!o.isMesh || !o.geometry) return;
        o.geometry.computeBoundingBox();
        const bb = o.geometry.boundingBox;
        for (let i = 0; i < 8; i++) {
          const p = new THREE.Vector3(
            i & 1 ? bb.max.x : bb.min.x,
            i & 2 ? bb.max.y : bb.min.y,
            i & 4 ? bb.max.z : bb.min.z).applyMatrix4(o.matrixWorld);
          const d2 = p.lengthSq();
          if (d2 > best) { best = d2; tip = p; }
        }
      });

      if (tip && tip.lengthSq() > 1e-12) {
        const tipDir = tip.clone().normalize();

        // 2) 薄面法向: 手掌(根 link 直属网格并集)在根坐标系下最薄轴
        const corners = [];
        built.root.children.forEach(c => {
          if (!c.isMesh || !c.geometry) return;
          c.geometry.computeBoundingBox();
          const bb = c.geometry.boundingBox;
          for (let i = 0; i < 8; i++) {
            corners.push(new THREE.Vector3(
              i & 1 ? bb.max.x : bb.min.x,
              i & 2 ? bb.max.y : bb.min.y,
              i & 4 ? bb.max.z : bb.min.z).applyMatrix4(c.matrixWorld));
          }
        });

        if (corners.length) {
          const ext = [0, 0, 0];
          [0, 1, 2].forEach(a => {
            const vs = corners.map(p => [p.x, p.y, p.z][a]);
            ext[a] = Math.max(...vs) - Math.min(...vs);
          });
          const ax = [ext[0], ext[1], ext[2]].indexOf(Math.min(ext[0], ext[1], ext[2]));
          const nL0 = new THREE.Vector3(0, 0, 0);
          nL0.setComponent(ax, 1);   // 薄面法向 (本地)

          // 3) 立起姿态: 用 base link 自身主轴刚性对齐世界坐标 —— 安装面严格平行于网格。
          //    注意: 不能用指尖方向 tipDir 当竖直轴 (指链关节偏移使指尖偏离 base z 轴约 9°, 会整体倾斜),
          //    tipDir 只用于确定手指轴的正负方向; O30 的 base 最长轴即手指轴(局部 z)。
          //    手指轴: base 主轴中与指尖方向最对齐的一根 (并列时避开薄轴), 符号取指向指尖侧
          const order = [0, 1, 2].sort((a, b) => Math.abs(tipDir.getComponent(b)) - Math.abs(tipDir.getComponent(a)));
          const fingerAx = (order[0] === ax) ? order[1] : order[0];
          const axes = [new THREE.Vector3(1, 0, 0), new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 1)];
          const e1 = axes[fingerAx].clone();
          if (tipDir.getComponent(fingerAx) < 0) e1.negate();
          const e2 = nL0.clone();   // 薄轴与手指轴是不同的坐标主轴, 天然正交
          // 本地基: e1=手指轴(→上), e2=薄面法向(→相机), e3=e1×e2
          const e3 = new THREE.Vector3().crossVectors(e1, e2).normalize();
          // 目标基: f1=上, f2=相机方位, f3=f1×f2
          const f1 = up.clone();
          const f2 = camDirH.clone();
          const f3 = new THREE.Vector3().crossVectors(f1, f2).normalize();

          // R = F * E^T  (把 e1->f1, e2->f2, e3->f3)
          const E = new THREE.Matrix4().makeBasis(e1, e2, e3);
          const F = new THREE.Matrix4().makeBasis(f1, f2, f3);
          const R = F.clone().multiply(E.transpose());
          built.root.quaternion.premultiply(new THREE.Quaternion().setFromRotationMatrix(R));
        } else {
          // 兜底: 指尖方向朝上
          built.root.quaternion.premultiply(new THREE.Quaternion().setFromUnitVectors(tipDir, up));
        }

        // 4) 底座贴地: 把根下移到手最低点落在 y=0 (此姿态下 hand_base_link 安装面即最低面, 稳贴网格)
        built.root.updateMatrixWorld(true);
        const base = new THREE.Box3().setFromObject(built.root);
        built.root.position.y -= base.min.y;
        built.root.updateMatrixWorld(true);
      }
    }

    // 相机自适应: 取景包住整只手
    const box = new THREE.Box3().setFromObject(built.root);
    const size = Math.max(box.getSize(new THREE.Vector3()).length(), 0.01);
    // 以质心(视觉重心)为观察中心而非包围盒中心: 手是非对称体(拇指一侧),
    // 用包围盒中心会把整体视觉重心偏向一侧, 看起来「向左/右偏移」
    const center = computeCOM(built.root, box.getCenter(new THREE.Vector3()));
    controls.target.copy(center);
    camera.position.copy(center).add(new THREE.Vector3(size * 0.62, size * 0.5, size * 0.62));
    camera.near = size / 100;
    camera.far = size * 100;
    camera.updateProjectionMatrix();

    const gridY = box.min.y - 0.002;   // 略低于底座, 避免共面闪烁
    const grid = new THREE.GridHelper(size * 4, 20, 0x35436b, 0x222c45);
    grid.position.y = gridY;
    scene.add(grid);

    // 接触阴影: 手座下方的柔和径向渐变暗盘, 从任意角度都能看出「手座贴着网格」
    {
      const bb = new THREE.Box3();
      built.root.children.forEach(c => {
        if (!c.isMesh || !c.geometry) return;
        c.geometry.computeBoundingBox();
        bb.union(new THREE.Box3().copy(c.geometry.boundingBox).applyMatrix4(c.matrixWorld));
      });
      if (!bb.isEmpty()) {
        const cv = document.createElement('canvas');
        cv.width = cv.height = 128;
        const ctx = cv.getContext('2d');
        const grad = ctx.createRadialGradient(64, 64, 6, 64, 64, 64);
        grad.addColorStop(0, 'rgba(0,0,0,0.5)');
        grad.addColorStop(0.55, 'rgba(0,0,0,0.28)');
        grad.addColorStop(1, 'rgba(0,0,0,0)');
        ctx.fillStyle = grad;
        ctx.fillRect(0, 0, 128, 128);
        const shadow = new THREE.Mesh(
          new THREE.CircleGeometry(Math.max(bb.max.x - bb.min.x, bb.max.z - bb.min.z) * 0.5 * 1.9, 48),
          new THREE.MeshBasicMaterial({ map: new THREE.CanvasTexture(cv), transparent: true, depthWrite: false }));
        shadow.rotation.x = -Math.PI / 2;   // 平铺在 XZ 平面
        shadow.position.set((bb.min.x + bb.max.x) / 2, gridY + 0.0008, (bb.min.z + bb.max.z) / 2);
        scene.add(shadow);
      }
    }
    controls.update();

    let meshCount = 0;
    built.root.traverse(o => { if (o.isMesh && o.geometry) meshCount++; });
    if (meshCount > 0) {
      const partialNote = meta.partial
        ? `  (⚠️ ${meta.missing.length} 个关节无 3D 映射: ${meta.missing.join(', ')})`
        : '';
      setStatus(`URDF 已加载: ${built.linkCount} links / ${Object.keys(joints).length} joints / ${meshCount} meshes  (hand=${meta.hand}, 拖拽旋转 / 滚轮缩放)${partialNote}`, 'ok');
      window.__hand_viewer_ok = true;
    } else {
      setStatus(`URDF 结构已加载但 0 个网格 (STL 全部加载失败, 请查控制台)`, 'err');
      window.__hand_viewer_ok = 'error';
    }
    resize();
    new ResizeObserver(resize).observe(container);
    animate();
  } catch (e) {
    console.error('[URDF viewer]', e);
    window.__hand_viewer_ok = 'error';
    setStatus(`URDF 加载失败: ${e && e.message || e}`, 'err');
  }
}

init();

/* 供测试/扩展使用 (浏览器中无副作用) */
export { buildUrdf, parseVec3, rpyToQuat };