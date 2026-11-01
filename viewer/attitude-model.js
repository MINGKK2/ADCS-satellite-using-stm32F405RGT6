import * as THREE from 'three';
import { OrbitControls } from './vendor/three/OrbitControls.js';
import { attitudeQuaternion } from './attitude-model-math.js';

const mount=document.getElementById('scene');
const status=document.getElementById('modelStatus');
const details=document.getElementById('modelDetails');
let ready=false,valid=false,received=false,error=false;
function showState(){
  if(error)return;
  status.textContent=!ready?'Đang tải mô hình CAD…':valid?'Tư thế đo từ EKF':received?'Mẫu tư thế cũ · giữ hình tại mẫu cuối':'Mô hình CAD · chờ dữ liệu tư thế';
  status.classList.toggle('model-live',ready&&valid);
}

try {
  const renderer=new THREE.WebGLRenderer({alpha:true,antialias:true,powerPreference:'high-performance'});
  renderer.setPixelRatio(Math.min(globalThis.devicePixelRatio||1,1.75));
  renderer.setClearColor(0x000000,0);
  renderer.outputColorSpace=THREE.SRGBColorSpace;
  renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.25;
  renderer.domElement.setAttribute('aria-label','Mô hình CubeSat 3D từ bản vẽ CAD; kéo để đổi góc nhìn');
  mount.appendChild(renderer.domElement);
  const scene=new THREE.Scene();
  const camera=new THREE.PerspectiveCamera(38,1,.05,100);
  camera.up.set(0,0,1); // CAD Z is up; roll X, pitch Y, yaw Z.
  const controls=new OrbitControls(camera,renderer.domElement);
  controls.enableDamping=true;controls.dampingFactor=.10;controls.enablePan=false;
  controls.target.set(0,0,0);
  const attitudeRoot=new THREE.Group();scene.add(attitudeRoot);
  const targetQuaternion=new THREE.Quaternion();
  const axes=new THREE.AxesHelper(1.9);axes.material.depthTest=false;axes.renderOrder=10;
  axes.visible=true;attitudeRoot.add(axes);
  const shells=[];let radius=2.5;

  scene.add(new THREE.HemisphereLight(0xd9edff,0x23313e,2.3));
  const key=new THREE.DirectionalLight(0xfff3df,4.0);key.position.set(-3,-4,7);scene.add(key);
  const fill=new THREE.DirectionalLight(0x7dbfff,2.0);fill.position.set(4,3,2);scene.add(fill);
  const rim=new THREE.DirectionalLight(0xcce6ff,2.2);rim.position.set(2,5,6);scene.add(rim);
  const grid=new THREE.GridHelper(8,32,0x41667f,0x214052);
  grid.rotation.x=Math.PI/2;grid.position.z=-1.65;
  grid.material.transparent=true;grid.material.opacity=.24;scene.add(grid);
  const floor=new THREE.Mesh(new THREE.CircleGeometry(2.25,64),new THREE.MeshBasicMaterial({color:0x010a11,transparent:true,opacity:.27,depthWrite:false}));
  floor.position.z=-1.63;scene.add(floor);

  function resetView(){
    const vertical=THREE.MathUtils.degToRad(camera.fov),horizontal=2*Math.atan(Math.tan(vertical/2)*camera.aspect);
    const distance=radius/Math.sin(Math.min(vertical,horizontal)/2)*(mount.clientWidth<480?1.6:1.38);
    camera.position.set(-.64,-.78,.58).normalize().multiplyScalar(distance);
    controls.minDistance=radius*1.25;controls.maxDistance=radius*7;
    controls.target.set(0,0,0);controls.update();
  }
  function resize(){
    const width=Math.max(1,mount.clientWidth),height=Math.max(1,mount.clientHeight);
    renderer.setSize(width,height,false);camera.aspect=width/height;camera.updateProjectionMatrix();
    camera.setViewOffset(width,height,0,height*(width<480?.085:.065),width,height);
    resetView();
  }
  new ResizeObserver(resize).observe(mount);resize();resetView();
  document.getElementById('modelReset').addEventListener('click',resetView);
  document.getElementById('modelAxes').addEventListener('click',event=>{
    axes.visible=!axes.visible;event.currentTarget.setAttribute('aria-pressed',String(axes.visible));
  });
  document.getElementById('modelShell').addEventListener('click',event=>{
    const open=event.currentTarget.getAttribute('aria-pressed')!=='true';
    for(const mesh of shells)for(const material of mesh.material){
      material.transparent=open;material.opacity=open?.16:1;material.depthWrite=!open;material.needsUpdate=true;
    }
    event.currentTarget.setAttribute('aria-pressed',String(open));
    event.currentTarget.textContent=open?'Hiện vỏ đầy đủ':'Nhìn bên trong';
  });
  globalThis.cubesatModel={
    setAttitude(state){
      const q=attitudeQuaternion(state.roll,state.pitch,state.yaw);
      if(q&&(state.valid||!received)){targetQuaternion.set(...q);if(!received)attitudeRoot.quaternion.copy(targetQuaternion);received=true;}
      valid=!!(state.valid&&q);
      showState();
    },
    setValidity(value){valid=!!value;showState();}
  };
  if(globalThis.cubesatAttitude)globalThis.cubesatModel.setAttitude(globalThis.cubesatAttitude);

  const base='./assets/cubesat/';
  const [description,geometryResponse]=await Promise.all([fetch(base+'cubesat-model.json'),fetch(base+'cubesat-model.bin')]);
  if(!description.ok||!geometryResponse.ok)throw new Error('Không tải được file mô hình');
  const manifest=await description.json(),buffer=await geometryResponse.arrayBuffer();
  if(buffer.byteLength!==manifest.binaryBytes)throw new Error('Dữ liệu mesh chưa đầy đủ');
  const model=new THREE.Group();
  for(const part of manifest.parts){
    const geometry=new THREE.BufferGeometry();
    geometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(buffer,part.positionOffset,part.vertexCount*3),3));
    geometry.setIndex(new THREE.BufferAttribute(new Uint32Array(buffer,part.indexOffset,part.indexCount),1));
    geometry.computeVertexNormals();geometry.computeBoundingSphere();
    const materials=part.groups.map((group,index)=>{
      geometry.addGroup(group.start,group.count,index);
      return new THREE.MeshStandardMaterial({color:group.color,metalness:part.kind==='wheel'?.48:part.kind==='frame'?.42:.18,
        roughness:part.kind==='wheel'?.38:.55,flatShading:true,side:THREE.DoubleSide,
        polygonOffset:part.kind==='pads',polygonOffsetFactor:-1,polygonOffsetUnits:-1});
    });
    const mesh=new THREE.Mesh(geometry,materials);mesh.name=part.name;model.add(mesh);
    if(part.shell)shells.push(mesh);
    if(part.kind==='frame'||part.kind==='wheel'){
      const edges=new THREE.LineSegments(new THREE.EdgesGeometry(geometry,35),new THREE.LineBasicMaterial({color:part.kind==='wheel'?0x43505a:0x263944,transparent:true,opacity:.55}));
      edges.name=part.name+' outlines';mesh.add(edges);
    }
  }
  attitudeRoot.add(model);
  radius=Math.hypot(...manifest.dimensionsMm)*manifest.cadToDisplayScale/2+.05;
  resetView();
  ready=true;showState();
  details.textContent=`${manifest.parts.length} chi tiết CAD · ${(manifest.triangles/1000).toFixed(0)}k tam giác · màu minh họa`;
  document.getElementById('modelShell').disabled=false;
  let last=0;
  function animate(time){
    requestAnimationFrame(animate);
    if(time-last<1000/30||document.hidden)return;
    const dt=last?Math.min(.1,(time-last)/1000):1/30;last=time;
    attitudeRoot.quaternion.slerp(targetQuaternion,1-Math.exp(-18*dt));
    controls.update();renderer.render(scene,camera);
  }
  requestAnimationFrame(animate);
  renderer.domElement.addEventListener('webglcontextlost',event=>{event.preventDefault();error=true;status.textContent='WebGL bị gián đoạn · tải lại trang để xem mô hình';});
} catch(cause){
  error=true;status.textContent='Không tải được mô hình 3D';
  details.textContent=(cause?.message||String(cause))+' · dữ liệu/điều khiển vẫn hoạt động.';
  console.error('CubeSat model:',cause);
}
