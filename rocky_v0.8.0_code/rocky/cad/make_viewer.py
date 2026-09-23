"""Self-contained HTML 3D viewer, v0.3 — the whole project, zero deps.

Modes: Full robot · Carapace v0 · Bench & field (stand stack, dock,
tools, clips) · Leg assembly · Hand · Parts: mechanism · Parts: session
4-5. Hand-rolled WebGL + orbit controls; STLs embedded base64; per-mode
hint text. Browser-verify before delivery (D007).
"""
import base64, os, json
import numpy as np
from stl import mesh as stlmesh

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")

def b64(name):
    with open(os.path.join(OUT, f"{name}.stl"), "rb") as f:
        return base64.b64encode(f.read()).decode()

def bbox(name):
    m = stlmesh.Mesh.from_file(os.path.join(OUT, f"{name}.stl"))
    v = m.vectors.reshape(-1, 3)
    return v.min(0), v.max(0)

# ---- parts modes: side-by-side along X, vertically centered ----
def parts_row(part_list, start_x=-170, gap=28):
    items, xoff = [], start_x
    for name, label, color in part_list:
        mn, mx = bbox(name)
        items.append(dict(label=label, color=color, data=b64(name),
                          offset=[xoff - float(mn[0]), 0.0,
                                  -float(mn[2] + mx[2]) / 2]))
        xoff += float(mx[0] - mn[0]) + gap
    return items


PARTS_MECH = [
    ("coxa_yaw_base", "Coxa yaw base", [0.49, 0.36, 0.75]),
    ("coxa_fork", "Coxa fork", [0.61, 0.48, 0.83]),
    ("coxa_crown_cap", "Crown cap (D046)", [0.66, 0.52, 0.84]),
    ("femur_link", "Femur link v0.2", [0.70, 0.62, 0.86]),
    ("horn_coupler", "Horn coupler", [0.76, 0.68, 0.88]),
    ("servo_blank", "Servo blank v0.2", [0.34, 0.33, 0.40]),
    ("hand_hub", "Hand hub", [0.55, 0.44, 0.79]),
    ("hand_finger", "Hand finger", [0.66, 0.52, 0.84]),
    ("hand_cam", "Hand cam disc", [0.76, 0.68, 0.88]),
    ("servo_st3215_dummy", "ST3215 servo (ref)", [0.32, 0.32, 0.38]),
]
PARTS_S45 = [
    ("body_deck", "Deck v0.4", [0.49, 0.36, 0.75]),
    ("battery_sled", "Battery sled (I5)", [0.61, 0.48, 0.83]),
    ("avionics_tray", "Avionics tray (I4)", [0.70, 0.62, 0.86]),
    ("busboard_bracket", "Bus star-board bracket", [0.55, 0.44, 0.79]),
    ("shell_sector", "Carapace sector", [0.66, 0.52, 0.84]),
    ("shell_cap", "Hatch cap", [0.76, 0.68, 0.88]),
    ("tube_clip", "Tube clip", [0.61, 0.48, 0.83]),
    ("link_clip", "Link clip", [0.70, 0.62, 0.86]),
]
parts_items = parts_row(PARTS_MECH)
parts45_items = parts_row(PARTS_S45, start_x=-260)

# bench & field: stand STACKED (base 0..35, section seats at 35, crown at
# 115 -> deck-bottom 127 config), dock beside it, tools + clips in front
bench_items = [
    dict(label="Stand base", color=[0.49, 0.36, 0.75],
         data=b64("stand_base"), offset=[0, 0, 0]),
    dict(label="Stand section", color=[0.55, 0.44, 0.79],
         data=b64("stand_section"), offset=[0, 0, 35]),
    dict(label="Stand crown", color=[0.66, 0.52, 0.84],
         data=b64("stand_crown"), offset=[0, 0, 115]),
    dict(label="Dock base", color=[0.61, 0.48, 0.83],
         data=b64("dock_base"), offset=[210, 0, 0]),
    dict(label="Dock tower", color=[0.76, 0.68, 0.88],
         data=b64("dock_tower"), offset=[256, 0, 4]),
    dict(label="I2 hook", color=[0.70, 0.62, 0.86],
         data=b64("tool_hook"), offset=[-60, -140, 20]),
    dict(label="I2 scoop", color=[0.66, 0.52, 0.84],
         data=b64("tool_scoop"), offset=[10, -140, 20]),
    dict(label="Clips", color=[0.55, 0.44, 0.79],
         data=b64("tube_clip"), offset=[80, -140, 0]),
]

MODES = [
    dict(id="full", label="Full robot", target=[0, 0, -25], radius=760, grid_z=-95,
         hint="Toggle the shells off to see the mechanism. The carapace is "
              "the real printable sector geometry, instanced 5x like the legs.",
         items=[
            dict(label="Shells / carapace", color=[0.55, 0.42, 0.75],
                 data=b64("preview_shells"), offset=[0, 0, 0]),
            dict(label="Skeleton / mechanics", color=[0.24, 0.24, 0.30],
                 data=b64("preview_skeleton"), offset=[0, 0, 0]),
         ]),
    dict(id="shell", label="Carapace v0", target=[0, 0, 25], radius=430, grid_z=-15,
         hint="Session 5: terraced rock tiers, leg arches, vent gills, LED "
              "channel, I6 bars, latch feet — and 72°-periodic seams with a "
              "chaining tongue-and-groove (print two sectors to feel it).",
         items=[
            dict(label="Assembled (5 sectors + cap)", color=[0.55, 0.42, 0.75],
                 data=b64("shell_ring_assembled"), offset=[0, 0, 0]),
            dict(label="One sector (the print)", color=[0.66, 0.52, 0.84],
                 data=b64("shell_sector"), offset=[190, 0, 0]),
            dict(label="Hatch cap", color=[0.76, 0.62, 0.88],
                 data=b64("shell_cap"), offset=[-40, 195, -45]),
         ]),
    dict(id="weekend", label="Print weekend", target=[70, 0, 15], radius=470,
         grid_z=-8,
         hint="D046 (2026-09-17): the leg chain dry-fit rebuilt around its "
              "joints — bolt-on crown cap + M3 axle, horn couplers in the "
              "femur hubs, lips + straps in every cradle, blanks v0.2 with "
              "nut slots. check_assembly.py proves the assembly ORDER, not "
              "just non-overlap. Print the J1/J2 coupons first.",
         items=[
            dict(label="Coxa base (I1 plate)", color=[0.49, 0.36, 0.75],
                 data=b64("coxa_yaw_base"), offset=[0, 0, 0]),
            dict(label="Coxa fork", color=[0.61, 0.48, 0.83],
                 data=b64("coxa_fork"), offset=[0, 0, 0]),
            dict(label="Servo blanks ×3", color=[0.34, 0.33, 0.40],
                 data=b64("dryfit_blanks_posed"), offset=[0, 0, 0]),
            dict(label="Femur link v0.2", color=[0.70, 0.62, 0.86],
                 data=b64("femur_link_posed"), offset=[0, 0, 0]),
            dict(label="Horn couplers ×2", color=[0.76, 0.68, 0.88],
                 data=b64("dryfit_couplers_posed"), offset=[0, 0, 0]),
            dict(label="Crown cap + straps", color=[0.66, 0.52, 0.84],
                 data=b64("dryfit_cap_straps_posed"), offset=[0, 0, 0]),
            dict(label="Knee carrier", color=[0.55, 0.44, 0.79],
                 data=b64("tibia_knee_carrier"), offset=[0, 0, 0]),
            dict(label="Hand (closed = foot)", color=[0.66, 0.52, 0.84],
                 data=b64("hand_assembly_closed"), offset=[230, -70, 30]),
            dict(label="Fit ladder v2", color=[0.76, 0.68, 0.88],
                 data=b64("fit_ladder"), offset=[-30, -160, 0]),
         ]),
    dict(id="bench", label="Bench & field", target=[110, -20, 40], radius=560,
         grid_z=-2,
         hint="Session 5b/c: the maintenance stand (127 mm config shown — "
              "stack joints are printed I6 dovetails), the walk-on charging "
              "dock (funnel rails, crouch-to-mate tower), and the first I2 "
              "bayonet tools + harness clips.",
         items=bench_items),
    dict(id="leg", label="Leg assembly", target=[55, 0, -5], radius=430, grid_z=-88,
         hint="One leg, yaw axis at origin: coxa base + fork, femur link, "
              "knee carrier, SEA cartridge, tube. The bench jig consumes the "
              "identical I1 port geometry.",
         items=[
            dict(label="Leg skeleton", color=[0.55, 0.44, 0.79],
                 data=b64("leg_skeleton_assembly"), offset=[0, 0, 0]),
         ]),
    dict(id="hand", label="Hand open/closed", target=[0, 0, 25], radius=260, grid_z=-40,
         hint="v0.2.1: spiral cam, 0-55° sweep, graze-free across 14 poses. "
              "Closed, the three fingers ARE the walking foot (canon).",
         items=[
            dict(label="Hand closed (= foot)", color=[0.55, 0.44, 0.79],
                 data=b64("hand_assembly_closed"), offset=[-55, 0, 0]),
            dict(label="Hand open", color=[0.66, 0.52, 0.84],
                 data=b64("hand_assembly_open"), offset=[55, 0, 0]),
         ]),
    dict(id="parts", label="Parts: mechanism", target=[30, 0, 0], radius=420,
         grid_z=-60,
         hint="The session 1-2 core printables. Every part regenerates from "
              "params.yaml; run_all_checks.py verifies the whole tree.",
         items=parts_items),
    dict(id="parts45", label="Parts: session 4-5", target=[20, 0, 0], radius=520,
         grid_z=-60,
         hint="Deck v0.4 (leg ports + shell strikes), battery sled, avionics "
              "tray, star-board bracket, carapace prints, harness clips.",
         items=parts45_items),
]

html = r"""<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Pebble CAD v0.2 — Project ROCKY</title>
<style>
 html,body{margin:0;height:100%;font-family:system-ui,sans-serif;background:#17141f;color:#e8e2f5;overflow:hidden}
 canvas{display:block}
 #hud{position:absolute;top:0;left:0;padding:14px 16px;z-index:5;max-width:400px}
 h1{font-size:16px;margin:0 0 2px} .sub{font-size:11px;color:#9a8fc0;margin-bottom:10px}
 .btn{display:inline-block;margin:3px 4px 3px 0;padding:6px 10px;border-radius:8px;font-size:12px;
      background:#2a2438;border:1px solid #453a63;cursor:pointer;user-select:none}
 .btn.on{background:#5b4a8a;border-color:#8d6fc9}
 #hint{position:absolute;bottom:10px;left:16px;font-size:11px;color:#8b80ad;z-index:5}
 #err{position:absolute;top:40%;width:100%;text-align:center;color:#e8a0a0;font-size:14px;display:none}
</style></head><body>
<canvas id="gl"></canvas>
<div id="hud">
 <h1>Pebble — CAD v0.3 <span style="font-size:11px;color:#9a8fc0">(session 6)</span></h1>
 <div class="sub">Project ROCKY · radial pentapod · drag orbit · wheel zoom · right-drag pan</div>
 <div id="modeBtns"></div>
 <div id="itemBtns"></div>
</div>
<div id="hint"></div>
<div id="err">This viewer needs WebGL. Download the file and open it in a normal browser tab.</div>
<script>
const MODES = __MODES__;

function mat4mul(a,b){const o=new Float32Array(16);
 for(let c=0;c<4;c++)for(let r=0;r<4;r++){let s=0;for(let k=0;k<4;k++)s+=a[k*4+r]*b[c*4+k];o[c*4+r]=s;}return o;}
function persp(fovy,asp,n,f){const t=1/Math.tan(fovy/2),o=new Float32Array(16);
 o[0]=t/asp;o[5]=t;o[10]=(f+n)/(n-f);o[11]=-1;o[14]=2*f*n/(n-f);return o;}
function lookAt(e,c,u){
 let zx=e[0]-c[0],zy=e[1]-c[1],zz=e[2]-c[2];let zl=Math.hypot(zx,zy,zz);zx/=zl;zy/=zl;zz/=zl;
 let xx=u[1]*zz-u[2]*zy,xy=u[2]*zx-u[0]*zz,xz=u[0]*zy-u[1]*zx;let xl=Math.hypot(xx,xy,xz);xx/=xl;xy/=xl;xz/=xl;
 const yx=zy*xz-zz*xy,yy=zz*xx-zx*xz,yz=zx*xy-zy*xx;
 return new Float32Array([xx,yx,zx,0, xy,yy,zy,0, xz,yz,zz,0,
   -(xx*e[0]+xy*e[1]+xz*e[2]),-(yx*e[0]+yy*e[1]+yz*e[2]),-(zx*e[0]+zy*e[1]+zz*e[2]),1]);}
function translate(x,y,z){const o=new Float32Array(16);o[0]=o[5]=o[10]=o[15]=1;o[12]=x;o[13]=y;o[14]=z;return o;}

function parseSTL(b64d){
  const bin=atob(b64d),n=bin.length,buf=new ArrayBuffer(n),u8=new Uint8Array(buf);
  for(let i=0;i<n;i++)u8[i]=bin.charCodeAt(i);
  const dv=new DataView(buf),tri=dv.getUint32(80,true);
  const pos=new Float32Array(tri*9),nor=new Float32Array(tri*9);
  let o=84;
  for(let t=0;t<tri;t++){
    const nx=dv.getFloat32(o,true),ny=dv.getFloat32(o+4,true),nz=dv.getFloat32(o+8,true);o+=12;
    for(let v=0;v<3;v++){
      const i=t*9+v*3;
      pos[i]=dv.getFloat32(o,true);pos[i+1]=dv.getFloat32(o+4,true);pos[i+2]=dv.getFloat32(o+8,true);o+=12;
      nor[i]=nx;nor[i+1]=ny;nor[i+2]=nz;
    }
    o+=2;
  }
  return {pos,nor,count:tri*3};
}

const canvas=document.getElementById('gl');
const gl=canvas.getContext('webgl',{antialias:true});
if(!gl){document.getElementById('err').style.display='block';}
else{
const VS=`attribute vec3 aP;attribute vec3 aN;uniform mat4 uMVP;uniform mat4 uM;
varying vec3 vN;varying vec3 vW;
void main(){gl_Position=uMVP*vec4(aP,1.0);vN=aN;vW=(uM*vec4(aP,1.0)).xyz;}`;
const FS=`precision mediump float;varying vec3 vN;varying vec3 vW;
uniform vec3 uColor;uniform vec3 uEye;uniform float uLine;
void main(){
  if(uLine>0.5){gl_FragColor=vec4(uColor,1.0);return;}
  vec3 N=normalize(vN);
  vec3 L1=normalize(vec3(0.5,0.35,0.8));
  vec3 L2=normalize(uEye-vW);
  float d=max(dot(N,L1),0.0)*0.55+max(dot(N,L2),0.0)*0.35;
  float hemi=0.30+0.20*(N.z*0.5+0.5);
  gl_FragColor=vec4(uColor*(hemi+d),1.0);}`;
function shader(src,type){const s=gl.createShader(type);gl.shaderSource(s,src);gl.compileShader(s);
 if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw gl.getShaderInfoLog(s);return s;}
const prog=gl.createProgram();
gl.attachShader(prog,shader(VS,gl.VERTEX_SHADER));gl.attachShader(prog,shader(FS,gl.FRAGMENT_SHADER));
gl.linkProgram(prog);gl.useProgram(prog);
const aP=gl.getAttribLocation(prog,'aP'),aN=gl.getAttribLocation(prog,'aN');
const uMVP=gl.getUniformLocation(prog,'uMVP'),uM=gl.getUniformLocation(prog,'uM'),
      uColor=gl.getUniformLocation(prog,'uColor'),uEye=gl.getUniformLocation(prog,'uEye'),
      uLine=gl.getUniformLocation(prog,'uLine');
gl.enable(gl.DEPTH_TEST);

function makeGrid(size,step){
  const v=[];
  for(let i=-size;i<=size;i+=step){v.push(i,-size,0, i,size,0, -size,i,0, size,i,0);}
  const arr=new Float32Array(v);
  const pb=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,pb);gl.bufferData(gl.ARRAY_BUFFER,arr,gl.STATIC_DRAW);
  return {pb,count:arr.length/3};
}
const grid=makeGrid(350,25);

for(const mode of MODES){
  for(const it of mode.items){
    const g=parseSTL(it.data);delete it.data;
    it.pb=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,it.pb);gl.bufferData(gl.ARRAY_BUFFER,g.pos,gl.STATIC_DRAW);
    it.nb=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,it.nb);gl.bufferData(gl.ARRAY_BUFFER,g.nor,gl.STATIC_DRAW);
    it.count=g.count;it.visible=true;
    it.model=translate(it.offset[0],it.offset[1],it.offset[2]);
  }
}

let mode=MODES[0];
let theta=-1.0,phi=1.15,radius=mode.radius,target=mode.target.slice(),eye=[0,0,0],drag=null;
function updateEye(){
  eye=[target[0]+radius*Math.sin(phi)*Math.cos(theta),
       target[1]+radius*Math.sin(phi)*Math.sin(theta),
       target[2]+radius*Math.cos(phi)];
}
const modeBox=document.getElementById('modeBtns'),itemBox=document.getElementById('itemBtns');
const hintBox=document.getElementById('hint');
function setHint(m){hintBox.textContent=m.hint||'';}
MODES.forEach((m,i)=>{
  const b=document.createElement('span');b.className='btn'+(i===0?' on':'');b.textContent=m.label;
  b.onclick=()=>{mode=m;radius=m.radius;target=m.target.slice();updateEye();
    [...modeBox.children].forEach((c,j)=>c.classList.toggle('on',MODES[j]===m));
    buildItemBtns();setHint(m);};
  modeBox.appendChild(b);
});
setHint(MODES[0]);
function buildItemBtns(){
  itemBox.innerHTML='';
  if(mode.items.length<2)return;
  mode.items.forEach(it=>{
    const b=document.createElement('span');b.className='btn'+(it.visible?' on':'');b.textContent=it.label;
    b.onclick=()=>{it.visible=!it.visible;b.classList.toggle('on');};
    itemBox.appendChild(b);
  });
}
buildItemBtns();

canvas.addEventListener('mousedown',e=>drag={x:e.clientX,y:e.clientY,btn:e.button});
addEventListener('mouseup',()=>drag=null);
canvas.addEventListener('contextmenu',e=>e.preventDefault());
addEventListener('mousemove',e=>{
  if(!drag)return;
  const dx=e.clientX-drag.x,dy=e.clientY-drag.y;drag.x=e.clientX;drag.y=e.clientY;
  if(drag.btn===2){
    const s=radius/700;
    target[0]+=(dx*Math.sin(theta)+dy*Math.cos(theta)*Math.cos(phi))*s;
    target[1]+=(-dx*Math.cos(theta)+dy*Math.sin(theta)*Math.cos(phi))*s;
    target[2]+=dy*s*Math.sin(phi);
  }else{theta-=dx*0.008;phi=Math.min(3.0,Math.max(0.15,phi-dy*0.008));}
  updateEye();
});
canvas.addEventListener('wheel',e=>{e.preventDefault();
  radius=Math.min(2200,Math.max(40,radius*(1+e.deltaY*0.001)));updateEye();},{passive:false});
let touches={};
canvas.addEventListener('touchstart',e=>{for(const t of e.changedTouches)touches[t.identifier]=[t.clientX,t.clientY];});
canvas.addEventListener('touchend',e=>{for(const t of e.changedTouches)delete touches[t.identifier];delete touches.pinch;});
canvas.addEventListener('touchmove',e=>{e.preventDefault();
  if(e.touches.length===1){
    const t=e.touches[0],p=touches[t.identifier]||[t.clientX,t.clientY];
    theta-=(t.clientX-p[0])*0.008;phi=Math.min(3.0,Math.max(0.15,phi-(t.clientY-p[1])*0.008));
    touches[t.identifier]=[t.clientX,t.clientY];updateEye();
  }else if(e.touches.length===2){
    const a=e.touches[0],b=e.touches[1];
    const d=Math.hypot(a.clientX-b.clientX,a.clientY-b.clientY);
    if(touches.pinch)radius=Math.min(2200,Math.max(40,radius*touches.pinch/d));
    touches.pinch=d;updateEye();
  }},{passive:false});

function draw(){
  const w=innerWidth,h=innerHeight;
  if(canvas.width!==w||canvas.height!==h){canvas.width=w;canvas.height=h;gl.viewport(0,0,w,h);}
  gl.clearColor(0.090,0.078,0.122,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
  const P=persp(0.9,w/h,1,8000),V=lookAt(eye,target,[0,0,1]),PV=mat4mul(P,V);
  gl.uniform3f(uEye,eye[0],eye[1],eye[2]);
  gl.uniform1f(uLine,1);gl.uniform3f(uColor,0.20,0.17,0.30);
  gl.uniformMatrix4fv(uMVP,false,mat4mul(PV,translate(0,0,mode.grid_z)));
  gl.uniformMatrix4fv(uM,false,translate(0,0,mode.grid_z));
  gl.bindBuffer(gl.ARRAY_BUFFER,grid.pb);gl.enableVertexAttribArray(aP);
  gl.vertexAttribPointer(aP,3,gl.FLOAT,false,0,0);
  gl.disableVertexAttribArray(aN);gl.vertexAttrib3f(aN,0,0,1);
  gl.drawArrays(gl.LINES,0,grid.count);
  gl.uniform1f(uLine,0);
  for(const it of mode.items){
    if(!it.visible)continue;
    gl.uniformMatrix4fv(uMVP,false,mat4mul(PV,it.model));
    gl.uniformMatrix4fv(uM,false,it.model);
    gl.uniform3f(uColor,it.color[0],it.color[1],it.color[2]);
    gl.bindBuffer(gl.ARRAY_BUFFER,it.pb);gl.enableVertexAttribArray(aP);
    gl.vertexAttribPointer(aP,3,gl.FLOAT,false,0,0);
    gl.bindBuffer(gl.ARRAY_BUFFER,it.nb);gl.enableVertexAttribArray(aN);
    gl.vertexAttribPointer(aN,3,gl.FLOAT,false,0,0);
    gl.drawArrays(gl.TRIANGLES,0,it.count);
  }
  requestAnimationFrame(draw);
}
updateEye();draw();
}
</script></body></html>
"""

html = html.replace("__MODES__", json.dumps(MODES))
path = os.path.join(HERE, "pebble_viewer.html")
with open(path, "w") as f:
    f.write(html)
print("viewer written:", path, f"({os.path.getsize(path)/1e6:.1f} MB)")
