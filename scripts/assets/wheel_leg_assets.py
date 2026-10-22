"""Model-specific armor replacement; source ZIP remains immutable."""
import hashlib
import numpy as np

SOURCE_SHA256='ef3ab28628b7315b7e3cfe5a38714b17bb24942506be2c8ec3666165de702be1'
REMOVED={f'part_{i:03d}' for a,b in [(74,89),(94,109),(513,528),(533,548)] for i in range(a,b+1)}
PANELS={'1_ARMOR_ROOT':('part_518',[0,1,0]), '2_ARMOR_ROOT':('part_538',[0,-1,0]), '3_ARMOR_ROOT':('part_079',[1,0,0]), '_ARMOR_ROOT':('part_099',[-1,0,0])}

def stl_vertices(z,filename):
    data=z.read(filename);count=int.from_bytes(data[80:84],'little')
    if len(data)!=84+count*50:raise ValueError('invalid binary STL')
    return np.frombuffer(data,dtype=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')]),offset=84,count=count)['v'].reshape(-1,3).astype(float)*0.001

def replacement(z,source,parts,cad_to_base):
    if hashlib.sha256(source).hexdigest()!=SOURCE_SHA256:raise ValueError('armor replacement requires the reviewed URDF archive')
    if not REMOVED<=parts.keys():raise ValueError('missing reviewed armor parts')
    mounts={}
    for name,(part,outward) in PANELS.items():
        points=np.unique(stl_vertices(z,parts[part]['mesh_file']),axis=0)
        center=points.mean(0);_,axes=np.linalg.eigh(np.cov((points-center).T));normal=axes[:,0]
        if normal@outward<0:normal=-normal
        up=np.array([0.,0.,1.]);up-=normal*(up@normal);up/=np.linalg.norm(up)
        horizontal=np.cross(up,normal)
        # CAD panel's outer plane, not the averaged solid/PCB center.
        u=points@horizontal;v=points@up;w=points@normal
        position=horizontal*((u.min()+u.max())*.5)+up*((v.min()+v.max())*.5)+normal*w.max()
        # Existing black module: local -Y is outward, +Z is upright, face lies at Y=0.
        matrix=np.eye(4);matrix[:3,:3]=np.column_stack([horizontal,-normal,up]);matrix[:3,3]=position
        m=cad_to_base@matrix
        spans=sorted(np.ptp((points-center)@axes,axis=0))
        if not (0.0028<spans[0]<0.0032 and 0.12<spans[1]<0.13 and 0.13<spans[2]<0.14):raise ValueError('reviewed armor panel geometry changed')
        mounts[name]={'panel_part':part,'matrix':m.T.reshape(-1).tolist(),'center_m':m[:3,3].tolist(),'normal':(-m[:3,1]).tolist(),'center_error_m':0.,'normal_error_deg':0.}
    return {'source_sha256':SOURCE_SHA256,'removed_parts':sorted(REMOVED),'preserved_mount_brackets':['part_072','part_073','part_092','part_093','part_511','part_512','part_531','part_532'],'armor_mounts':mounts}