import assert from 'node:assert/strict';
import fs from 'node:fs';
import crypto from 'node:crypto';
import { attitudeQuaternion } from './attitude-model-math.js';
import { Quaternion,Euler,Vector3 } from './vendor/three/three.core.min.js';
for(const angles of [[0,0,0],[90,0,0],[0,90,0],[0,0,90],[30,-40,120],[-170,89,-179]]){
  const actual=attitudeQuaternion(...angles),expected=new Quaternion().setFromEuler(new Euler(...angles.map(v=>v*Math.PI/180),'ZYX'));
  expected.toArray().forEach((v,i)=>assert(Math.abs(v-actual[i])<1e-12));
  assert(Math.abs(actual.reduce((s,v)=>s+v*v,0)-1)<1e-12);
}
assert.equal(attitudeQuaternion(NaN,0,0),null);
const rotated=new Vector3(1,0,0).applyQuaternion(new Quaternion(...attitudeQuaternion(0,0,90)));
assert(Math.abs(rotated.y-1)<1e-12&&Math.abs(rotated.x)<1e-12);
const midpoint=new Quaternion(...attitudeQuaternion(0,0,179)).slerp(new Quaternion(...attitudeQuaternion(0,0,-179)),.5);
assert(Math.abs(midpoint.w)<1e-12); // shortest path goes through 180, not zero
const manifest=JSON.parse(fs.readFileSync(new URL('./assets/cubesat/cubesat-model.json',import.meta.url)));
const data=fs.readFileSync(new URL('./assets/cubesat/cubesat-model.bin',import.meta.url));
assert.equal(data.length,manifest.binaryBytes);assert.equal(manifest.parts.length,24);
assert.equal(crypto.createHash('sha256').update(data).digest('hex'),manifest.binarySHA256);
let triangles=0;
for(const part of manifest.parts){
  assert.equal(part.indexCount%3,0);assert.equal(part.positionOffset%4,0);assert.equal(part.indexOffset%4,0);
  let end=0;
  for(const group of part.groups){assert.equal(group.start,end);end+=group.count;}
  assert.equal(end,part.indexCount);
  for(let i=0;i<part.vertexCount*3;i++){const v=data.readFloatLE(part.positionOffset+i*4);assert(Number.isFinite(v)&&Math.abs(v)<4);}
  for(let i=0;i<part.indexCount;i++)assert(data.readUInt32LE(part.indexOffset+i*4)<part.vertexCount);
  triangles+=part.indexCount/3;
}
assert.equal(manifest.parts.filter(p=>p.kind==='wheel').length,3);
assert.equal(manifest.parts.filter(p=>p.kind==='motor').length,3);
assert.equal(triangles,manifest.triangles);
console.log(`PASS CAD assembly: 24 parts, ${triangles} triangles, ${(data.length/1048576).toFixed(2)} MiB, indexed buffers and checksum`);
console.log('PASS measured ZYX quaternion, axis sign, invalid input, shortest interpolation across ±180°');
