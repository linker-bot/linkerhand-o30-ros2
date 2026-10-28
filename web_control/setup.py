import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'web_control'


def _urdf_assets():
    '''动态枚举 urdf/ 下的全部资源 (型号/左右手/meshes 结构不限, 通用多型号支持)。'''
    assets = []
    base = os.path.join('web_control', 'urdf')
    if not os.path.isdir(base):
        return assets
    for dirpath, _dirnames, filenames in os.walk(base):
        for fn in filenames:
            if fn.startswith('.'):
                continue
            rel = os.path.relpath(os.path.join(dirpath, fn), 'web_control')
            assets.append(rel.replace(os.sep, '/'))
    return assets


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: [
        'static/*.html',
        'static/js/*.js',
        'static/lib/three/build/*.js',
        'static/lib/three/examples/jsm/controls/*.js',
        'static/lib/three/examples/jsm/loaders/*.js',
    ] + _urdf_assets()},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'static'),
            glob(os.path.join('web_control', 'static', '*.html'))),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linkerhand',
    maintainer_email='linkerhand@todo.todo',
    description='灵巧手 Web 控制台 (通用: 关节滑动条 + 温度 + URDF 3D 显示, 关节名/数量/映射均自适应)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'web_control = web_control.web_control:main',
        ],
    },
)
