#!/usr/bin/env python3
"""Turn a fusion2URDF export of a two-wheeled robot into a package you can
simulate in Gazebo Classic and drive with diff_drive_controller.

    python3 fusion2sim.py ~/Documents/test_ws/src/myrobot_description
    python3 fusion2sim.py PKG --no-controller        # Day 4 stage: no ros2_control yet
    python3 fusion2sim.py PKG --left LJOINT --right RJOINT --lidar LINK

Works on the package in place. The exported urdf/, launch/, config/ and
build files are first copied to fusion_export_backup/.

What it does, all measured from the exported meshes, so any size works:
  - wheels: the two continuous joints whose child link name contains
    "wheel" (or --left/--right). Radius, width and separation come from the
    wheel meshes. Axes are flipped where needed so both turn about +y.
  - joints that share a name with a link get a "_joint" suffix
    (Gazebo rejects the duplicate).
  - collision: cylinders for wheels, a box for each other part, and a
    frictionless sphere wherever the chassis touches the ground (a ball
    caster), placed exactly at wheel-bottom height so the robot stands level.
  - adds Gazebo friction, a 2D LiDAR (on the link named like "lidar", or a
    new one on top), an IMU, a world, launch files and the controller files.

Assumptions: x forward, z up (REP 103), wheel axles along y, meshes in
metres, one root link. It prints what it found; check those numbers.
"""
import argparse
import math
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
TEMPLATE = HERE / 'template'


# ---------------------------------------------------------------- geometry
def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = map(lambda f, a: f(a), [math.cos, math.sin] * 3, [r, r, p, p, y, y])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def matrix_rpy(m):
    p = math.asin(max(-1.0, min(1.0, -m[2, 0])))
    if abs(math.cos(p)) < 1e-9:
        return 0.0, p, math.atan2(-m[0, 1], m[1, 1])
    return math.atan2(m[2, 1], m[2, 2]), p, math.atan2(m[1, 0], m[0, 0])


def vec(s, default='0 0 0'):
    return np.array([float(v) for v in (s or default).split()])


def fmt(v):
    return ' '.join(f'{x:.6g}' if abs(x) > 1e-12 else '0' for x in v)


def dae_vertices(path):
    """All vertex positions of a COLLADA file, in metres."""
    root = ET.parse(path).getroot()
    tag = lambda e: e.tag.rsplit('}', 1)[-1]
    scale, out, has_matrix = 1.0, [], False
    for e in root.iter():
        t = tag(e)
        if t == 'unit':
            scale = float(e.get('meter', '1'))
        elif t == 'matrix':
            has_matrix = True
        elif t == 'source' and 'position' in (e.get('id') or '').lower():
            fa = next((c for c in e if tag(c) == 'float_array'), None)
            if fa is not None and fa.text:
                out.append(np.array(fa.text.split(), float).reshape(-1, 3))
    if not out:
        raise SystemExit(f'no vertices found in {path}')
    if has_matrix:
        print(f'  note: {path.name} has node transforms; they are ignored (fusion2URDF bakes them)')
    return np.vstack(out) * scale


# ---------------------------------------------------------------- model
class Link:
    def __init__(self, el, pkg_dir):
        self.name = el.get('name')
        self.el = el
        vis = el.find('visual')
        self.mesh = None
        self.vis_xyz = np.zeros(3)
        self.vis_rpy = np.zeros(3)
        if vis is not None:
            o = vis.find('origin')
            if o is not None:
                self.vis_xyz, self.vis_rpy = vec(o.get('xyz')), vec(o.get('rpy'))
            m = vis.find('geometry/mesh')
            if m is not None:
                fn = m.get('filename')
                rel = re.sub(r'^package://[^/]+/', '', fn)
                self.mesh_uri = fn
                self.mesh = pkg_dir / rel
                self.mesh_scale = vec(m.get('scale'), '1 1 1')
            mat = vis.find('material')
            self.material = mat.get('name') if mat is not None else None
        self.verts = None   # vertices in the link frame

    def load(self):
        if self.mesh is not None and self.mesh.exists():
            v = dae_vertices(self.mesh) * self.mesh_scale
            self.verts = (rpy_matrix(*self.vis_rpy) @ v.T).T + self.vis_xyz
        return self.verts


class Joint:
    def __init__(self, el):
        self.el = el
        self.name = el.get('name')
        self.type = el.get('type')
        self.parent = el.find('parent').get('link')
        self.child = el.find('child').get('link')
        o = el.find('origin')
        self.xyz = vec(o.get('xyz') if o is not None else None)
        self.rpy = vec(o.get('rpy') if o is not None else None)
        a = el.find('axis')
        self.axis = vec(a.get('xyz') if a is not None else None, '1 0 0')


def load_model(pkg, urdf_dir):
    urdfs = sorted(urdf_dir.glob('*.urdf'))
    if not urdfs:
        raise SystemExit('no urdf/*.urdf in the package: is this a fusion2URDF export?')
    tree = ET.parse(urdfs[0])
    root = tree.getroot()
    links = {e.get('name'): Link(e, pkg) for e in root.findall('link')}
    joints = [Joint(e) for e in root.findall('joint')]
    materials = [e for e in root.findall('material')]
    children = {j.child for j in joints}
    roots = [n for n in links if n not in children]
    if len(roots) != 1:
        raise SystemExit(f'expected one root link, found {roots}')
    return root.get('name'), links, joints, materials, roots[0]


def poses_in_base(links, joints, base):
    """4x4 transform of every link frame in the base frame (joints at zero)."""
    T = {base: np.eye(4)}
    by_parent = {}
    for j in joints:
        by_parent.setdefault(j.parent, []).append(j)
    stack = [base]
    while stack:
        p = stack.pop()
        for j in by_parent.get(p, []):
            t = np.eye(4)
            t[:3, :3] = rpy_matrix(*j.rpy)
            t[:3, 3] = j.xyz
            T[j.child] = T[p] @ t
            stack.append(j.child)
    return T


def to_base(T, pts):
    return (T[:3, :3] @ pts.T).T + T[:3, 3]


# ---------------------------------------------------------------- analysis
def analyse(args, links, joints, base):
    T = poses_in_base(links, joints, base)
    for l in links.values():
        l.load()
    report = []

    # wheels
    movable = [j for j in joints if j.type in ('continuous', 'revolute')]
    if args.left and args.right:
        wj = [j for j in joints if j.name in (args.left, args.right)]
    else:
        wj = [j for j in movable if 'wheel' in j.child.lower() and j.parent == base]
    if len(wj) != 2:
        names = ', '.join(f'{j.name} ({j.child})' for j in movable)
        raise SystemExit(f'found {len(wj)} wheel joints; give --left and --right. Movable joints: {names}')

    wheels = []
    for j in wj:
        l = links[j.child]
        if l.verts is None:
            raise SystemExit(f'wheel link {l.name} has no mesh to measure')
        a = j.axis / np.linalg.norm(j.axis)
        c = (l.verts.min(0) + l.verts.max(0)) / 2
        d = l.verts - c
        along = d @ a
        width = along.max() - along.min()
        radius = np.linalg.norm(d - np.outer(along, a), axis=1).max()
        centre_b = to_base(T[l.name], c[None])[0]
        axis_b = T[l.name][:3, :3] @ a
        wheels.append(dict(joint=j, link=l, centre=c, centre_b=centre_b, axis=a, axis_b=axis_b,
                           width=width, radius=radius))
    wheels.sort(key=lambda w: -w['centre_b'][1])          # left wheel first (+y)
    left, right = wheels
    if args.left and args.right:   # the user knows best which is which
        left, right = sorted(wheels, key=lambda w: w['joint'].name != args.left)
    for w in wheels:
        if abs(w['axis_b'][1]) < 0.9:
            raise SystemExit(f"{w['joint'].name}: axle points {fmt(w['axis_b'])} in {base}; "
                             'expected along y. Is x forward and z up in the CAD?')
        w['flip'] = w['axis_b'][1] < 0
    radius = (left['radius'] + right['radius']) / 2
    separation = abs(left['centre_b'][1] - right['centre_b'][1])
    ground = min(w['centre_b'][2] - w['radius'] for w in wheels)
    wheel_x = (left['centre_b'][0] + right['centre_b'][0]) / 2
    report += [f'wheels: left {left["joint"].name}, right {right["joint"].name}',
               f'  radius {radius:.4f} m, width {left["width"]:.4f} m, separation {separation:.4f} m',
               f'  wheel centres at x = {wheel_x:+.4f}, ground at z = {ground:+.4f} in {base}']
    for w in wheels:
        if w['flip']:
            report.append(f'  {w["joint"].name}: axis flipped so it turns about +y like the other wheel')

    # casters: anything other than the wheels that reaches the ground
    wheel_links = {left['link'].name, right['link'].name}
    contacts = []
    for l in links.values():
        if l.name in wheel_links or l.verts is None:
            continue
        vb = to_base(T[l.name], l.verts)
        low = vb[vb[:, 2] < ground + args.ground_tol]
        if len(low):
            contacts.append((l.name, low))
    casters = []
    for lname, low in contacts:
        pts = low[:, :2]
        clusters = []
        for p in pts:   # greedy clustering in xy
            for cl in clusters:
                if np.linalg.norm(cl['c'] - p) < 0.03:
                    cl['pts'].append(p); cl['c'] = np.mean(cl['pts'], 0); break
            else:
                clusters.append({'c': p.copy(), 'pts': [p]})
        for cl in clusters:
            casters.append((lname, cl['c']))
    if not casters:
        bv = to_base(np.eye(4), links[base].verts) if links[base].verts is not None else np.zeros((1, 3))
        front = bv[:, 0].max() - 2 * args.caster_radius
        casters = [(base, np.array([front if wheel_x < 0 else bv[:, 0].min() + 2 * args.caster_radius, 0.0]))]
        report.append('  WARNING: nothing but the wheels reaches the ground; added a support sphere '
                      f'at x = {casters[0][1][0]:+.3f}. Add a caster in CAD for a real robot.')
    for lname, c in casters:
        report.append(f'caster contact at x = {c[0]:+.4f}, y = {c[1]:+.4f} (from {lname}); '
                      f'frictionless sphere r = {args.caster_radius} at wheel-bottom height')
    if all(abs(c[0] - wheel_x) < 0.02 for _, c in casters):
        report.append('  WARNING: casters are in line with the wheels; the robot will tip.')

    # renames
    link_names = set(links)
    renames = {j.name: j.name + '_joint' for j in joints if j.name in link_names}
    for old, new in renames.items():
        report.append(f'joint {old} -> {new} (same name as a link; Gazebo rejects that)')

    # lidar
    lidar = args.lidar or next((n for n in links if re.search('lidar|laser|scan', n, re.I)), None)
    report.append(f'lidar: {"link " + lidar if lidar else "none in CAD, adding lidar_link on top"}')

    mass = sum(float(l.el.find('inertial/mass').get('value')) for l in links.values()
               if l.el.find('inertial/mass') is not None)
    report.append(f'total mass {mass:.3f} kg')
    return dict(T=T, left=left, right=right, radius=radius, separation=separation, ground=ground,
                wheel_x=wheel_x, casters=casters, renames=renames, lidar=lidar, mass=mass,
                wheel_links=wheel_links, contact_links={n for n, _ in contacts}, report=report)


# ---------------------------------------------------------------- writers
def collision_xml(shape, xyz, rpy, ind):
    p = ' ' * ind
    return (f'{p}<collision>\n{p}  <origin xyz="{fmt(xyz)}" rpy="{fmt(rpy)}"/>\n'
            f'{p}  <geometry>\n{p}    {shape}\n{p}  </geometry>\n{p}</collision>\n')


def write_assembly(a, robot, links, joints, base, out, args):
    v_lim = max(10.0, 1.0 / a['radius'])                       # at least 1 m/s at the rim
    effort = max(1.5, 4 * a['mass'] * 9.81 * a['radius'])
    s = [f'<?xml version="1.0"?>\n<!-- Generated by fusion2sim.py from the fusion2URDF export.\n'
         + '\n'.join('     ' + r for r in a['report']) + '\n-->\n'
         f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{robot}_macro">\n'
         f'  <xacro:macro name="{robot}" params="prefix">\n\n']
    for l in links.values():
        e = l.el
        s.append(f'    <link name="${{prefix}}{l.name}">\n')
        inr = e.find('inertial')
        if inr is not None:
            s.append('    ' + ET.tostring(inr, encoding='unicode').strip().replace('\n', '\n    ') + '\n')
        vis = e.find('visual')
        if vis is not None:
            s.append('      ' + ET.tostring(vis, encoding='unicode').strip().replace('\n', '\n  ') + '\n')
        # collisions (unnamed, so Gazebo applies the friction blocks)
        if l.name in a['wheel_links']:
            w = a['left'] if l.name == a['left']['link'].name else a['right']
            z = np.array([0, 0, 1.0])
            ax = w['axis']
            v = np.cross(z, ax)
            if np.linalg.norm(v) < 1e-9:
                rot = np.eye(3) if ax[2] > 0 else rpy_matrix(math.pi, 0, 0)
            else:
                k = v / np.linalg.norm(v); ang = math.acos(max(-1, min(1, z @ ax)))
                K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
                rot = np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K
            s.append(collision_xml(f'<cylinder radius="{w["radius"]:.6g}" length="{w["width"]:.6g}"/>',
                                   w['centre'], matrix_rpy(rot), 6))
        elif l.verts is not None:
            vb = l.verts
            if l.name in a['contact_links'] or l.name == base:
                # keep the body clear of the ground; casters do the touching
                T = a['T'][l.name]
                zb = to_base(T, vb)[:, 2]
                vb = vb[zb > a['ground'] + max(args.ground_tol, 0.004)]
            if len(vb):
                lo, hi = vb.min(0), vb.max(0)
                s.append(collision_xml(f'<box size="{fmt(np.maximum(hi - lo, 1e-3))}"/>', (lo + hi) / 2,
                                       (0, 0, 0), 6))
        if l.name == base:
            for _, c in a['casters']:
                r = args.caster_radius
                s.append('      <!-- caster: frictionless, its bottom exactly at wheel-bottom height -->\n')
                s.append(collision_xml(f'<sphere radius="{r}"/>', (c[0], c[1], a['ground'] + r), (0, 0, 0), 6))
        s.append('    </link>\n\n')
    for j in joints:
        name = a['renames'].get(j.name, j.name)
        w = next((w for w in (a['left'], a['right']) if w['joint'] is j), None)
        axis = -j.axis if (w and w['flip']) else j.axis
        s.append(f'    <joint name="${{prefix}}{name}" type="{j.type}">\n'
                 f'      <parent link="${{prefix}}{j.parent}"/>\n      <child link="${{prefix}}{j.child}"/>\n'
                 f'      <origin xyz="{fmt(j.xyz)}" rpy="{fmt(j.rpy)}"/>\n')
        if j.type != 'fixed':
            s.append(f'      <axis xyz="{fmt(axis)}"/>\n')
        if w:
            s.append(f'      <limit effort="{effort:.3g}" velocity="{v_lim:.3g}"/>\n')
        elif j.type in ('continuous', 'revolute', 'prismatic'):
            lim = j.el.find('limit')
            lo = f' lower="{lim.get("lower")}" upper="{lim.get("upper")}"' if (lim is not None and lim.get('lower')) else ''
            s.append(f'      <limit effort="1.0" velocity="10.0"{lo}/>\n')
        s.append('    </joint>\n\n')
    s.append('  </xacro:macro>\n</robot>\n')
    (out / 'urdf' / 'assemblies').mkdir(parents=True, exist_ok=True)
    (out / 'urdf' / 'assemblies' / f'{robot}.urdf.xacro').write_text(''.join(s))
    a['v_lim'], a['effort'] = v_lim, effort


def write_main(a, robot, pkg, materials, out):
    mats = ''.join('  ' + ET.tostring(m, encoding='unicode').strip() + '\n' for m in materials)
    (out / 'urdf' / f'{robot}.urdf.xacro').write_text(f'''<?xml version="1.0"?>
<!-- Entry point. Generated by fusion2sim.py.
       xacro urdf/{robot}.urdf.xacro                -> RViz model
       xacro urdf/{robot}.urdf.xacro use_sim:=true  -> plus Gazebo parts and sensors -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="{robot}">
  <xacro:arg name="use_sim" default="false"/>
  <xacro:arg name="prefix"  default=""/>
  <!-- set by rsp.launch.py: true when urdf/{robot}.ros2_control.xacro exists -->
  <xacro:arg name="use_control" default="false"/>
  <xacro:property name="use_sim" value="$(arg use_sim)"/>
  <xacro:property name="prefix"  value="$(arg prefix)"/>
{mats}
  <xacro:include filename="$(find {pkg})/urdf/assemblies/{robot}.urdf.xacro"/>
  <xacro:include filename="$(find {pkg})/urdf/{robot}.gazebo.xacro"/>
  <xacro:include filename="$(find {pkg})/urdf/{robot}.sensors.xacro"/>
  <xacro:if value="$(arg use_control)">
    <xacro:include filename="$(find {pkg})/urdf/{robot}.ros2_control.xacro"/>
  </xacro:if>

  <xacro:{robot} prefix="${{prefix}}"/>
  <xacro:if value="$(arg use_control)">
    <xacro:{robot}_ros2_control prefix="${{prefix}}" use_sim="${{use_sim}}"/>
  </xacro:if>
  <xacro:if value="${{use_sim}}">
    <xacro:{robot}_gazebo  prefix="${{prefix}}"/>
    <xacro:{robot}_sensors prefix="${{prefix}}"/>
  </xacro:if>
</robot>
''')


def write_gazebo(a, robot, pkg, base, out):
    # contact stiffness grows with weight, or a heavy robot sinks into the ground
    k = max(1.0, a['mass'] / 5.0)
    surf = f'<kp>{1e6 * k:.4g}</kp><kd>{100 * k:.4g}</kd><minDepth>0.001</minDepth>'
    grip = '<mu1>1.0</mu1><mu2>1.0</mu2>' + surf
    slide = '<mu1>0.0</mu1><mu2>0.0</mu2>' + surf
    wheels = ''.join(f'''    <gazebo reference="${{prefix}}{w['link'].name}">{grip}<material>Gazebo/DarkGrey</material></gazebo>\n'''
                     for w in (a['left'], a['right']))
    (out / 'urdf' / f'{robot}.gazebo.xacro').write_text(f'''<?xml version="1.0"?>
<!-- Gazebo Classic plugin and contact surfaces. Generated by fusion2sim.py. -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="{robot}_gazebo" params="prefix">
    <xacro:if value="$(arg use_control)">
    <gazebo>
      <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
        <parameters>$(find {pkg})/config/ros2_controllers.yaml</parameters>
      </plugin>
    </gazebo>
    </xacro:if>
    <!-- tyres grip -->
{wheels}    <!-- the base carries the caster sphere(s): they slide -->
    <gazebo reference="${{prefix}}{base}">{slide}</gazebo>
  </xacro:macro>
</robot>
''')


def write_sensors(a, robot, links, base, out):
    bv = links[base].verts if links[base].verts is not None else np.zeros((1, 3))
    if a['lidar']:
        lv = links[a['lidar']].verts
        cz = (lv[:, 2].min() + lv[:, 2].max()) / 2 if lv is not None else 0.0
        cxy = ((lv.min(0) + lv.max(0)) / 2)[:2] if lv is not None else (0, 0)
        lidar_link = f'''    <link name="${{prefix}}laser_frame"/>
    <joint name="${{prefix}}laser_joint" type="fixed">
      <parent link="${{prefix}}{a['lidar']}"/>
      <child link="${{prefix}}laser_frame"/>
      <origin xyz="{fmt([cxy[0], cxy[1], cz])}" rpy="0 0 0"/>
    </joint>
'''
    else:
        top = bv[:, 2].max()
        lidar_link = f'''    <link name="${{prefix}}laser_frame">
      <inertial><mass value="0.05"/><inertia ixx="1e-05" ixy="0" ixz="0" iyy="1e-05" iyz="0" izz="1e-05"/></inertial>
      <visual><geometry><cylinder radius="0.02" length="0.03"/></geometry></visual>
      <collision><geometry><cylinder radius="0.02" length="0.03"/></geometry></collision>
    </link>
    <joint name="${{prefix}}laser_joint" type="fixed">
      <parent link="${{prefix}}{base}"/>
      <child link="${{prefix}}laser_frame"/>
      <origin xyz="0 0 {top + 0.015:.4f}" rpy="0 0 0"/>
    </joint>
'''
    n_av = '<noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise>'
    n_la = '<noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise>'
    # the LiDAR must not see the robot itself: start the rays just outside the body
    span = np.linalg.norm(np.ptp(bv[:, :2], 0)) / 2
    rmin = max(0.12, round(span + 0.02, 2))
    (out / 'urdf' / f'{robot}.sensors.xacro').write_text(f'''<?xml version="1.0"?>
<!-- Simulated 2D LiDAR and IMU. Generated by fusion2sim.py. -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="{robot}_sensors" params="prefix">
{lidar_link}
    <link name="${{prefix}}imu_link">
      <inertial><mass value="0.001"/><inertia ixx="1e-09" ixy="0" ixz="0" iyy="1e-09" iyz="0" izz="1e-09"/></inertial>
    </link>
    <joint name="${{prefix}}imu_joint" type="fixed">
      <parent link="${{prefix}}{base}"/>
      <child link="${{prefix}}imu_link"/>
      <origin xyz="0 0 {bv[:, 2].max() + 0.002:.4f}" rpy="0 0 0"/>
    </joint>

    <gazebo reference="${{prefix}}laser_frame">
      <sensor name="${{prefix}}lidar" type="ray">
        <always_on>true</always_on>
        <visualize>true</visualize>
        <update_rate>10.0</update_rate>
        <ray>
          <scan><horizontal>
            <samples>360</samples><resolution>1.0</resolution>
            <min_angle>-3.141593</min_angle><max_angle>3.141593</max_angle>
          </horizontal></scan>
          <range><min>{rmin}</min><max>3.5</max><resolution>0.015</resolution></range>
          <noise><type>gaussian</type><mean>0.0</mean><stddev>0.01</stddev></noise>
        </ray>
        <plugin name="${{prefix}}lidar_controller" filename="libgazebo_ros_ray_sensor.so">
          <ros><remapping>~/out:=scan</remapping></ros>
          <output_type>sensor_msgs/LaserScan</output_type>
          <frame_name>${{prefix}}laser_frame</frame_name>
        </plugin>
      </sensor>
    </gazebo>

    <gazebo reference="${{prefix}}imu_link">
      <sensor name="${{prefix}}imu" type="imu">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <imu>
          <angular_velocity><x>{n_av}</x><y>{n_av}</y><z>{n_av}</z></angular_velocity>
          <linear_acceleration><x>{n_la}</x><y>{n_la}</y><z>{n_la}</z></linear_acceleration>
        </imu>
        <plugin name="${{prefix}}imu_plugin" filename="libgazebo_ros_imu_sensor.so">
          <ros><remapping>~/out:=imu/data</remapping></ros>
          <frame_name>${{prefix}}imu_link</frame_name>
          <initial_orientation_as_reference>false</initial_orientation_as_reference>
        </plugin>
      </sensor>
    </gazebo>
  </xacro:macro>
</robot>
''')


def write_control(a, robot, joints, out):
    lname = a['renames'].get(a['left']['joint'].name, a['left']['joint'].name)
    rname = a['renames'].get(a['right']['joint'].name, a['right']['joint'].name)
    others = [a['renames'].get(j.name, j.name) for j in joints
              if j.type in ('continuous', 'revolute', 'prismatic') and j not in (a['left']['joint'], a['right']['joint'])]
    v = a['v_lim']
    wheel = lambda n: f'''      <joint name="${{prefix}}{n}">
        <command_interface name="velocity">
          <param name="min">{-v:.3g}</param><param name="max">{v:.3g}</param>
        </command_interface>
        <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
        <state_interface name="velocity"/>
      </joint>
'''
    passive = ''.join(f'''      <joint name="${{prefix}}{n}">   <!-- read only, never commanded -->
        <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
        <state_interface name="velocity"/>
      </joint>
''' for n in others)
    (out / 'urdf' / f'{robot}.ros2_control.xacro').write_text(f'''<?xml version="1.0"?>
<!-- Controller file. Tells ros2_control which joints it drives. Generated by fusion2sim.py. -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="{robot}_ros2_control" params="prefix use_sim">
    <ros2_control name="${{prefix}}{robot}_system" type="system">
      <hardware>
        <xacro:if value="${{use_sim}}"><plugin>gazebo_ros2_control/GazeboSystem</plugin></xacro:if>
        <xacro:unless value="${{use_sim}}">
          <plugin>mock_components/GenericSystem</plugin>
          <param name="calculate_dynamics">true</param>
        </xacro:unless>
      </hardware>
{wheel(lname)}{wheel(rname)}{passive}    </ros2_control>
  </xacro:macro>
</robot>
''')
    vmax = round(min(1.0, 0.8 * v * a['radius']), 2)
    wmax = round(min(3.0, 2 * vmax / a['separation']), 2)
    (out / 'config').mkdir(exist_ok=True)
    (out / 'config' / 'ros2_controllers.yaml').write_text(f'''# Controller file: joint_state_broadcaster + diff_drive_controller. Generated by fusion2sim.py.
controller_manager:
  ros__parameters:
    update_rate: 100
    use_sim_time: true
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

joint_state_broadcaster:
  ros__parameters:
    use_sim_time: true

diff_drive_controller:
  ros__parameters:
    use_sim_time: true
    left_wheel_names: ["{lname}"]
    right_wheel_names: ["{rname}"]
    wheel_separation: {a['separation']:.4f}   # measured from the wheel meshes
    wheel_radius: {a['radius']:.4f}
    publish_rate: 50.0
    odom_frame_id: odom
    base_frame_id: base_link
    pose_covariance_diagonal:  [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    open_loop: false
    position_feedback: true
    enable_odom_tf: true
    cmd_vel_timeout: 0.5
    use_stamped_vel: false
    linear.x.has_velocity_limits: true
    linear.x.max_velocity: {vmax}
    linear.x.min_velocity: {-vmax}
    linear.x.has_acceleration_limits: true
    linear.x.max_acceleration: 1.0
    linear.x.min_acceleration: -1.0
    angular.z.has_velocity_limits: true
    angular.z.max_velocity: {wmax}
    angular.z.min_velocity: {-wmax}
    angular.z.has_acceleration_limits: true
    angular.z.max_acceleration: 3.0
    angular.z.min_acceleration: -3.0
''')


def copy_templates(pkg_name, robot, out, base):
    for src in TEMPLATE.rglob('*'):
        if src.is_dir():
            continue
        dst = out / src.relative_to(TEMPLATE)
        dst.parent.mkdir(parents=True, exist_ok=True)
        text = src.read_text().replace('@PKG@', pkg_name).replace('@ROBOT@', robot)
        if base != 'base_link':
            text = text.replace('base_link', base)
        dst.write_text(text)
        if dst.suffix == '.py' and 'scripts' in dst.parts:
            dst.chmod(0o755)


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('package', type=Path, help='the exported package folder (it is changed in place)')
    ap.add_argument('--left', help='left wheel joint name (default: found from the link names)')
    ap.add_argument('--right', help='right wheel joint name')
    ap.add_argument('--lidar', help='link that carries the LiDAR (default: a link named like lidar/laser)')
    ap.add_argument('--no-controller', action='store_true',
                    help='leave out the ros2_control files (add them later by running again)')
    ap.add_argument('--caster-radius', type=float, default=0.005, help='caster sphere radius, m')
    ap.add_argument('--ground-tol', type=float, default=0.004,
                    help='how close to the wheel bottoms a part must reach to count as a caster, m')
    args = ap.parse_args()

    pkg = args.package.resolve()
    xml = (pkg / 'package.xml')
    if not xml.exists():
        raise SystemExit(f'{pkg} has no package.xml')
    pkg_name = ET.parse(xml).getroot().findtext('name')
    backup = pkg / 'fusion_export_backup'
    if not backup.exists():   # keep the very first export, never overwrite it
        backup.mkdir()
        for name in ('urdf', 'launch', 'config', 'rviz', 'CMakeLists.txt', 'package.xml'):
            p = pkg / name
            if p.is_dir():
                shutil.copytree(p, backup / name)
            elif p.exists():
                shutil.copy2(p, backup / name)
    # always read the original export, so running twice gives the same result
    robot, links, joints, materials, base = load_model(pkg, backup / 'urdf')

    print(f'{pkg_name}: robot "{robot}", root link {base}, {len(links)} links, {len(joints)} joints')
    a = analyse(args, links, joints, base)
    for line in a['report']:
        print('  ' + line)

    for p in (pkg / 'urdf').glob('*.xacro'):
        p.unlink()
    write_assembly(a, robot, links, joints, base, pkg, args)
    write_main(a, robot, pkg_name, materials, pkg)
    write_gazebo(a, robot, pkg_name, base, pkg)
    write_sensors(a, robot, links, base, pkg)
    copy_templates(pkg_name, robot, pkg, base)
    write_control(a, robot, joints, pkg)
    if args.no_controller:
        (pkg / 'urdf' / f'{robot}.ros2_control.xacro').unlink()
        print('  controller files left out (--no-controller)')
    print(f'  wheel limits: {a["v_lim"]:.3g} rad/s, {a["effort"]:.3g} N m')

    # self-check: both URDF variants must expand and parse. $(find pkg) needs a
    # built package, so expand a temporary copy with the path written in.
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        for d in ('urdf', 'config', 'meshes'):
            shutil.copytree(pkg / d, Path(tmp) / d)
        for f in Path(tmp, 'urdf').rglob('*.xacro'):
            f.write_text(f.read_text().replace(f'$(find {pkg_name})', tmp))
        for extra in ([], ['use_sim:=true', 'use_control:=' + ('false' if args.no_controller else 'true')]):
            try:
                x = subprocess.run(['xacro', f'{tmp}/urdf/{robot}.urdf.xacro'] + extra,
                                   capture_output=True, text=True)
            except FileNotFoundError:
                print('  (xacro not found: source ROS 2 to run the self-check)')
                break
            if x.returncode:
                print('  xacro FAILED:', x.stderr.strip()[-400:])
                sys.exit(1)
            c = subprocess.run(['check_urdf', '/dev/stdin'], input=x.stdout, capture_output=True, text=True)
            tree = [l.strip() for l in c.stdout.splitlines() if 'root Link' in l]
            print(f'  check_urdf {" ".join(extra) or "(display)"}: {"OK" if c.returncode == 0 else "FAILED"}',
                  tree[0] if tree else c.stderr[-300:])
            if c.returncode:
                sys.exit(1)
    print(f'\nDone. Build and run:\n  cd <your workspace> && colcon build --symlink-install && source install/setup.bash\n'
          f'  ros2 launch {pkg_name} display.launch.py\n  ros2 launch {pkg_name} sim.launch.py rviz:=true')


if __name__ == '__main__':
    main()
