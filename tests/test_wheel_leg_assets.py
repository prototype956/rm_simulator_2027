"""Independent generated-asset regression; read-only and numpy-only."""
import unittest
import hashlib, json, struct, zipfile, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

class WheelLegAssetTests(unittest.TestCase):

    def test_generated_assets(self):
        r = Path(__file__).resolve().parents[1]
        d = r / 'assets/robots/wheel_leg'
        source = (d / 'source.zip').read_bytes()
        assert hashlib.sha256(source).hexdigest() == 'ef3ab28628b7315b7e3cfe5a38714b17bb24942506be2c8ec3666165de702be1'
        removed = {f'part_{i:03d}' for a, b in [(74, 89), (94, 109), (513, 528), (533, 548)] for i in range(a, b + 1)}
        z = zipfile.ZipFile(d / 'source.zip')
        self.addCleanup(z.close)
        original = ET.fromstring(z.read('robot.urdf'))
        derived = ET.parse(d / 'robot.urdf').getroot()
        parts = {v['part_id']: v for v in json.loads(z.read('parts.json'))['parts']}
        files = lambda xml, tag: {Path(n.attrib['filename']).name.split('_solid_')[0] for n in xml.findall(f'link/{tag}/geometry/mesh')}
        for tag in ['visual', 'collision']:
            assert files(original, tag) - files(derived, tag) == removed
            assert files(derived, tag) == files(original, tag) - removed
        for v in derived.findall('link/visual/material/color'):
            assert np.allclose(list(map(float, v.get('rgba').split())), [14 / 255] * 3 + [1])
        b = (d / 'wheel_leg.glb').read_bytes()
        n = struct.unpack_from('<I', b, 12)[0]
        g = json.loads(b[20:20 + n])
        binary = b[28 + n:]
        nodes = {x['name']: x for x in g['nodes'] if 'name' in x}

        def transform(n):
            if 'matrix' in n:
                return np.array(n['matrix']).reshape(4, 4).T
            x, y, z, w = n.get('rotation', [0, 0, 0, 1])
            m = np.eye(4)
            m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
            m[:3, :3] = m[:3, :3] @ np.diag(n.get('scale', [1, 1, 1]))
            m[:3, 3] = n.get('translation', [0, 0, 0])
            return m

        def values(i):
            a = g['accessors'][i]
            v = g['bufferViews'][a['bufferView']]
            return np.frombuffer(binary, dtype={5126: '<f4', 5125: '<u4', 5123: '<u2'}[a['componentType']], offset=v.get('byteOffset', 0) + a.get('byteOffset', 0), count=a['count'] * {'VEC3': 3, 'SCALAR': 1}[a['type']]).reshape(a['count'], -1)
        for name, part, outward in [('1_ARMOR_ROOT', 'part_518', [0, 1, 0]), ('2_ARMOR_ROOT', 'part_538', [0, -1, 0]), ('3_ARMOR_ROOT', 'part_079', [1, 0, 0]), ('_ARMOR_ROOT', 'part_099', [-1, 0, 0])]:
            data = z.read(parts[part]['mesh_file'])
            count = struct.unpack_from('<I', data, 80)[0]
            tri = np.frombuffer(data, dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]), offset=84, count=count)['v'].astype(float) * 0.001
            cross = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
            normal = cross[np.argmax(np.linalg.norm(cross, axis=1))]
            normal /= np.linalg.norm(normal)
            if normal @ outward < 0:
                normal = -normal
            assert normal @ outward > 0.95
            pts = tri.reshape(-1, 3)
            up = np.array([0.0, 0.0, 1.0])
            up -= normal * (normal @ up)
            up /= np.linalg.norm(up)
            h = np.cross(up, normal)
            cad = h * ((pts @ h).min() + (pts @ h).max()) * 0.5 + up * ((pts @ up).min() + (pts @ up).max()) * 0.5 + normal * (pts @ normal).max()
            expected = np.array([cad[0], cad[2] - 0.182232674, -cad[1]])
            expected_normal = np.array([normal[0], normal[2], -normal[1]])
            root = nodes[name]
            armor = next((g['nodes'][i] for i in root['children'] if g['nodes'][i].get('name') == name.replace('_ROOT', '')))
            primitive = g['meshes'][armor['mesh']]['primitives'][0]
            pts = values(primitive['attributes']['POSITION'])
            m = transform(armor)
            pts = pts @ m[:3, :3].T + m[:3, 3]
            local = np.array([(pts[:, 0].min() + pts[:, 0].max()) * 0.5, pts[:, 1].min(), (pts[:, 2].min() + pts[:, 2].max()) * 0.5])
            mount = transform(root)
            actual = mount[:3, :3] @ local + mount[:3, 3]
            actual_normal = -mount[:3, 1]
            error = np.linalg.norm(actual - expected)
            angle = np.degrees(np.arccos(np.clip(actual_normal @ expected_normal, -1, 1)))
            assert error <= 0.001 and angle <= 0.5, (name, error, angle)
        material = next((x for x in g['materials'] if x.get('name', '').startswith('URDF black')))['pbrMetallicRoughness']
        linear = np.array(material['baseColorFactor'][:3])
        srgb = np.where(linear <= 0.0031308, 12.92 * linear, 1.055 * linear ** (1 / 2.4) - 0.055) * 255
        assert np.allclose(srgb, 14, atol=0.01)
        rig = json.loads((d / 'rig.json').read_text())
        base = next((x for x in rig['links'] if x['name'] == 'base_link'))
        assert all((x['name'] not in removed for x in base['collision_parts']))
        assert len(base['collision_parts']) > 200
        assert set(['part_072', 'part_073', 'part_092', 'part_093', 'part_511', 'part_512', 'part_531', 'part_532']) <= files(derived, 'visual')
if __name__ == '__main__':
    unittest.main()
