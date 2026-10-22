"""Convert supplied wheel-leg URDF ZIP to Bevy GLB + kinematic metadata.
Only numpy is required. No Blender/ROS/runtime URDF dependency.
Usage: python scripts/assets/import_wheel_leg.py path/to/export.zip
"""
from pathlib import Path, PurePosixPath
import argparse, copy, hashlib, json, math, shutil, struct, zipfile
import xml.etree.ElementTree as ET
import numpy as np
from wheel_leg_assets import REMOVED, replacement

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'assets/robots/wheel_leg'

def origin(element):
    m = np.eye(4)
    if element is None:
        return m
    r, p, y = map(float, element.get('rpy', '0 0 0').split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    m[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    m[:3, 3] = list(map(float, element.get('xyz', '0 0 0').split()))
    return m

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    args = parser.parse_args()
    source = args.archive.read_bytes()
    with zipfile.ZipFile(args.archive) as z:
        # References are read directly from the ZIP; no archive extraction or package URLs.
        for name in z.namelist():
            p = PurePosixPath(name)
            if p.is_absolute() or '..' in p.parts or '\\' in name:
                raise ValueError(f'unsafe ZIP member: {name}')
        xml = z.read('robot.urdf')
        robot = ET.fromstring(xml)
        report = json.loads(z.read('export_report.json'))
        parts={p['part_id']:p for p in json.loads(z.read('parts.json'))['parts']}
        if report.get('errors') or report.get('coordinate_frame') != 'z_up_x_forward':
            raise ValueError('unsupported/invalid export coordinates')
        old = (ROOT/'assets/vehicle.glb').read_bytes()
        size = struct.unpack_from('<I', old, 12)[0]
        gltf = json.loads(old[20:20+size])
        bin_size = struct.unpack_from('<I', old, 20+size)[0]
        binary = bytearray(old[28+size:28+size+bin_size])
        nodes = gltf['nodes']
        named = {n.get('name'): i for i, n in enumerate(nodes)}
        # CAD wheel axes are X. CAD +Y (bore direction) becomes Bevy -Z, Z becomes +Y.
        # Recenter around CAD X=0 rather than the exported arbitrary base origin.
        convert = np.array([[1.,0,0,0.054449997], [0,0,1,0], [0,-1,0,0], [0,0,0,1]])
        basis = convert.copy(); basis[:3,3] = 0
        first_visual=robot.find("link[@name='base_link']/visual/origin")
        replacements=replacement(z,source,parts,convert@origin(first_visual))
        for link in robot.findall('link'):
            for tag in ['visual','collision']:
                for element in list(link.findall(tag)):
                    mesh=element.find('geometry/mesh')
                    if mesh is not None and Path(mesh.get('filename')).name.split('_solid_')[0] in REMOVED:
                        link.remove(element)
        for visual in robot.findall('link/visual'):
            material=visual.find('material')
            if material is None:material=ET.SubElement(visual,'material',{'name':'structure_black'})
            color=material.find('color')
            if color is None:color=ET.SubElement(material,'color')
            color.set('rgba','0.0549019608 0.0549019608 0.0549019608 1')
        xml=ET.tostring(robot,encoding='utf-8',xml_declaration=True)
        joints = []
        zero = {'base_link': convert}
        for j in robot.findall('joint'):
            child, parent = j.find('child').get('link'), j.find('parent').get('link')
            local = origin(j.find('origin'))
            zero[child] = zero[parent] @ local
            joints.append({'name': j.get('name'), 'parent': parent, 'child': child,
                           'origin': local.T.reshape(-1).tolist(),
                           'axis': list(map(float,j.find('axis').get('xyz').split())),
                           'lower': float(j.find('limit').get('lower')), 'upper': float(j.find('limit').get('upper')),
                           'velocity': float(j.find('limit').get('velocity'))})
        metadata = {'archive_sha256': hashlib.sha256(source).hexdigest(), 'conversion': convert.T.reshape(-1).tolist(),
                    'joints': joints, 'links': [], 'warnings': report.get('warnings',[]),
                    'gimbal_adapter': 'Existing yaw/pitch controller; URDF yaw bracket follows yaw only; pitch assembly follows the bore. Not a torque-driven serial gimbal.'}
        mat = len(gltf.setdefault('materials', []))
        gltf['materials'].append({'name':'URDF black matched to vehicle MAT_1', 'pbrMetallicRoughness':{'baseColorFactor':[0.004391442,0.004391442,0.004391442,1.0], 'metallicFactor':175/255,'roughnessFactor':238/255}, 'doubleSided':False})
        def add_node(n):
            nodes.append(n); return len(nodes)-1
        def accessor(a, kind):
            nonlocal binary
            a = np.asarray(a, dtype='<u4' if kind=='SCALAR' else '<f4')
            binary += b'\0' * (-len(binary)%4)
            view = len(gltf['bufferViews'])
            gltf['bufferViews'].append({'buffer':0,'byteOffset':len(binary),'byteLength':a.nbytes,'target':34963 if kind=='SCALAR' else 34962})
            binary.extend(a.tobytes())
            idx = len(gltf['accessors'])
            gltf['accessors'].append({'bufferView':view,'componentType':5125 if kind=='SCALAR' else 5126,'count':len(a),'type':kind, 'min':np.atleast_1d(a.min(axis=0)).tolist(), 'max':np.atleast_1d(a.max(axis=0)).tolist()})
            return idx
        link_nodes = {}
        total_triangles = 0
        for link in robot.findall('link'):
            name = link.get('name'); vertices=[]; normals=[]; collision_parts=[]
            for v in link.findall('visual'):
                mesh = v.find('geometry/mesh'); data = z.read(mesh.get('filename'))
                count = struct.unpack_from('<I',data,80)[0]
                if len(data) != 84+count*50:
                    raise ValueError('only binary STL is supported')
                triangles = np.frombuffer(data,dtype=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')]),offset=84,count=count)['v'].astype(float)
                scale=np.array(list(map(float,mesh.get('scale','1 1 1').split())))
                t=origin(v.find('origin')); a=triangles.reshape(-1,3)*scale; a=a@t[:3,:3].T+t[:3,3]
                tri=a.reshape(-1,3,3); n=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]); length=np.linalg.norm(n,axis=1); valid=length>1e-14
                tri=tri[valid]; n=n[valid]/length[valid,None]
                if name=='base_link' and np.ptp(a,axis=0).max()>=0.025 and np.ptp(a,axis=0).min()>0.00025:
                    _,ci=np.unique(np.floor(a/0.003).astype(int),axis=0,return_index=True)
                    ci=np.unique(np.concatenate([ci,a.argmin(axis=0),a.argmax(axis=0)]))
                    collision_parts.append({'name':Path(mesh.get('filename')).name.split('_solid_')[0], 'points':a[ci].round(8).tolist()})
                vertices.append(tri.reshape(-1,3));normals.append(np.repeat(n,3,axis=0)); total_triangles+=len(tri)
            a=np.concatenate(vertices); n=np.concatenate(normals)
            # Each link keeps its URDF-local geometry. Joints remain separate GLB nodes.
            packed, inverse=np.unique(np.concatenate([a,n],axis=1).astype('<f4'),axis=0,return_inverse=True)
            pos=accessor(packed[:,:3],'VEC3'); normal=accessor(packed[:,3:],'VEC3'); index=accessor(inverse,'SCALAR')
            mi=len(gltf['meshes']);gltf['meshes'].append({'name':'URDF_'+name,'primitives':[{'attributes':{'POSITION':pos,'NORMAL':normal},'material':mat,'indices':index}]})
            ni=add_node({'name':'URDF_'+name,'mesh':mi});link_nodes[name]=ni
            _, indices=np.unique(np.floor(a/0.02).astype(int),axis=0,return_index=True)
            # Include directional extrema so decimation cannot shrink the outer envelope.
            ids=np.unique(np.concatenate([indices,a.argmin(axis=0),a.argmax(axis=0)]))
            metadata['links'].append({'name':name,'collision_parts':collision_parts,'hull_points':a[ids].round(8).tolist(), 'zero_matrix':zero[name].T.reshape(-1).tolist(), 'bounds_min':a.min(0).tolist(),'bounds_max':a.max(0).tolist()})
            if name=='base_link':
                points=a@zero[name][:3,:3].T+zero[name][:3,3]
                supports=[]
                for sx in [-1,1]:
                    for sz in [-1,1]:
                        subset=points[(points[:,0]*sx>0.055)&(points[:,2]*sz>0.18)]
                        bottom=subset[:,1].min();tip=subset[subset[:,1]<bottom+0.001]
                        supports.append([float(tip[:,0].mean()),float(bottom),float(tip[:,2].mean())])
                metadata['auxiliary_supports']=supports
        nodes[link_nodes['base_link']]['matrix']=convert.T.reshape(-1).tolist()
        nodes[named['BASE']]['children']=[i for i in nodes[named['BASE']]['children'] if i!=named['CHASSIS']]+[link_nodes['base_link']]
        # Retain marker names required by setup; old chassis/turret meshes are removed.
        nodes[named['CHASSIS']].pop('mesh',None)
        nodes[named['BASE']]['children'].append(named['CHASSIS'])
        for joint in joints:
            child=link_nodes[joint['child']];parent=link_nodes[joint['parent']]
            nodes[child]['matrix']=joint['origin']
            nodes[parent].setdefault('children',[]).append(child)
        # Detach turret links from BASE. Use exact zero-pose geometry in the legacy
        # control frame, with no mesh scaling. A runtime yaw mount avoids pitching the bracket.
        yaw=link_nodes['link_008'];pitch=link_nodes['link_009']
        nodes[link_nodes['base_link']]['children'].remove(yaw)
        nodes[yaw]['children'].remove(pitch)
        nodes[yaw]['matrix']=zero['link_008'].T.reshape(-1).tolist()
        nodes[named['BASE']]['children'].append(yaw)
        pivot=zero['link_009'][:3,3]
        gimbal=nodes[named['GIMBAL']];gimbal.clear();gimbal.update({'name':'GIMBAL','translation':pivot.tolist(),'children':[named['GIMBAL_MAIN'],named['CAM_DIRECTION'],named['SHOT_DIRECTION']]})
        main=nodes[named['GIMBAL_MAIN']];main.clear();main.update({'name':'GIMBAL_MAIN','children':[pitch]})
        local=zero['link_009'].copy();local[:3,3]-=pivot;nodes[pitch]['matrix']=local.T.reshape(-1).tolist()
        # Explicit provisional bore/camera calibration (CAD +Y). Kept separate in metadata.
        calibration={'muzzle': [0.0,0.155,-0.20], 'camera':[0.045,0.19,-0.10], 'armor_centers':[[0.,-0.08,-0.315],[0.,-0.08,0.315],[0.325,-0.08,0.0],[-0.325,-0.08,0.0]]}
        for marker,key in [('SHOT_DIRECTION','muzzle'),('CAM_DIRECTION','camera')]:
            nodes[named[marker]]={'name':marker,'translation':calibration[key], 'rotation':[-math.sqrt(0.5),0,0,math.sqrt(0.5)] if key=='muzzle' else [0,0,0,1]}
        calibration_path = OUT/'calibration.json'
        if calibration_path.exists():
            calibration=json.loads(calibration_path.read_text(encoding='utf-8'))
        for marker,key in [('SHOT_DIRECTION','muzzle'),('CAM_DIRECTION','camera')]:
            nodes[named[marker]]['translation']=calibration[key]
        armor_nodes=[named[n] for n in ['1_ARMOR_ROOT','2_ARMOR_ROOT','3_ARMOR_ROOT','_ARMOR_ROOT']]
        for ni in armor_nodes:
            name=nodes[ni]['name'];m=replacements['armor_mounts'][name]['matrix']
            nodes[ni].pop('translation',None);nodes[ni].pop('rotation',None);nodes[ni]['matrix']=m
        calibration['armor_centers']=[replacements['armor_mounts'][nodes[ni]['name']]['center_m'] for ni in armor_nodes]
        calibration['armor_matrices']={k:v['matrix'] for k,v in replacements['armor_mounts'].items()}
        metadata['calibration']=calibration
        all_min=[];all_max=[]
        for l in metadata['links']:
            m=np.array(l['zero_matrix']).reshape(4,4).T
            corners=np.array([[x,y,z,1] for x in [l['bounds_min'][0],l['bounds_max'][0]] for y in [l['bounds_min'][1],l['bounds_max'][1]] for z in [l['bounds_min'][2],l['bounds_max'][2]]])@m.T
            all_min.append(corners[:,:3].min(0));all_max.append(corners[:,:3].max(0))
        metadata['root_height_m']=float(-np.min(all_min,axis=0)[1]+0.005)
        metadata['triangle_count']=total_triangles
        # Drop unreachable legacy chassis/calibration meshes and repack only live views.
        reachable=set()
        def visit(i):
            if i in reachable:return
            reachable.add(i)
            for c in nodes[i].get('children',[]):visit(c)
        for scene in gltf['scenes']:
            for i in scene['nodes']:visit(i)
        live_meshes={nodes[i]['mesh'] for i in reachable if 'mesh' in nodes[i]}
        live_accessors=set()
        for i in live_meshes:
            for primitive in gltf['meshes'][i]['primitives']:
                live_accessors.update(primitive['attributes'].values())
                if 'indices' in primitive:live_accessors.add(primitive['indices'])
        live_views={gltf['accessors'][i]['bufferView'] for i in live_accessors}
        live_views.update(i['bufferView'] for i in gltf.get('images',[]) if 'bufferView' in i)
        packed_binary=bytearray();view_map={};views=[]
        for i in sorted(live_views):
            v=copy.deepcopy(gltf['bufferViews'][i]);start=v.get('byteOffset',0)
            packed_binary+=b'\0'*(-len(packed_binary)%4);v['byteOffset']=len(packed_binary)
            packed_binary.extend(binary[start:start+v['byteLength']]);view_map[i]=len(views);views.append(v)
        accessor_map={i:n for n,i in enumerate(sorted(live_accessors))}
        accessors=[]
        for i in sorted(live_accessors):
            a=copy.deepcopy(gltf['accessors'][i]);a['bufferView']=view_map[a['bufferView']];accessors.append(a)
        mesh_map={i:n for n,i in enumerate(sorted(live_meshes))};meshes=[]
        for i in sorted(live_meshes):
            mesh=copy.deepcopy(gltf['meshes'][i])
            for primitive in mesh['primitives']:
                primitive['attributes']={k:accessor_map[v] for k,v in primitive['attributes'].items()}
                if 'indices' in primitive:primitive['indices']=accessor_map[primitive['indices']]
            meshes.append(mesh)
        node_map={i:n for n,i in enumerate(sorted(reachable))};new_nodes=[]
        for i in sorted(reachable):
            node=copy.deepcopy(nodes[i])
            if 'mesh' in node:node['mesh']=mesh_map[node['mesh']]
            if 'children' in node:node['children']=[node_map[c] for c in node['children']]
            new_nodes.append(node)
        for scene in gltf['scenes']:scene['nodes']=[node_map[i] for i in scene['nodes']]
        for image in gltf.get('images',[]):
            if 'bufferView' in image:image['bufferView']=view_map[image['bufferView']]
        gltf.update(nodes=new_nodes,meshes=meshes,accessors=accessors,bufferViews=views)
        binary=packed_binary
        binary+=b'\0'*(-len(binary)%4);gltf['buffers']=[{'byteLength':len(binary)}]
        js=json.dumps(gltf,separators=(',',':')).encode();js+=b' '*(-len(js)%4)
        result=struct.pack('<III',0x46546c67,2,28+len(js)+len(binary))+struct.pack('<II',len(js),0x4e4f534a)+js+struct.pack('<II',len(binary),0x004e4942)+binary
        OUT.mkdir(parents=True,exist_ok=True)
        (OUT/'wheel_leg.glb').write_bytes(result)
        (OUT/'replacement.json').write_text(json.dumps(replacements,indent=2),encoding='utf-8')
        (OUT/'rig.json').write_text(json.dumps(metadata,separators=(',',':')),encoding='utf-8')
        (OUT/'calibration.json').write_text(json.dumps(calibration,indent=2),encoding='utf-8')
        (OUT/'robot.urdf').write_bytes(xml)
        (OUT/'export_report.json').write_bytes(z.read('export_report.json'))
        (OUT/'source.zip').write_bytes(source)
        print(json.dumps({k:v for k,v in metadata.items() if k not in ['links','joints','conversion']},indent=2))
if __name__=='__main__':main()