// VPP four-bar search. World-at-rest coords (m): ground y=0, rear axle extended E0=(0,0.35)
const E0=[0,0.35];
const rot=(v,a)=>[v[0]*Math.cos(a)-v[1]*Math.sin(a),v[0]*Math.sin(a)+v[1]*Math.cos(a)];
const sub=(a,b)=>[a[0]-b[0],a[1]-b[1]],add=(a,b)=>[a[0]+b[0],a[1]+b[1]],len=v=>Math.hypot(v[0],v[1]),ang=v=>Math.atan2(v[1],v[0]);
function circ(c0,r0,c1,r1,ref){const d=len(sub(c1,c0));if(d>r0+r1||d<Math.abs(r0-r1))return null;const a=(r0*r0-r1*r1+d*d)/(2*d),h=Math.sqrt(Math.max(0,r0*r0-a*a));const u=[(c1[0]-c0[0])/d,(c1[1]-c0[1])/d];const p=[c0[0]+a*u[0],c0[1]+a*u[1]];const s1=[p[0]-h*u[1],p[1]+h*u[0]],s2=[p[0]+h*u[1],p[1]-h*u[0]];return len(sub(s1,ref))<len(sub(s2,ref))?s1:s2}
function sweep(g,n=120){const{A,l1,p0,B,l2,q0,l3,al,beta,sl}=g;const C0=add(A,[l1*Math.cos(p0),l1*Math.sin(p0)]),D0=add(B,[l2*Math.cos(q0),l2*Math.sin(q0)]);
const dCD=len(sub(D0,C0)),aCD0=ang(sub(D0,C0)),eL=rot(sub(E0,C0),-aCD0);
const G0=add(B,rot([l3,0],q0+al)),F=add(G0,[sl*Math.cos(beta),sl*Math.sin(beta)]);
const out=[];let Dp=D0;for(let i=0;i<=n;i++){const p=p0+g.dp*i/n,C=add(A,[l1*Math.cos(p),l1*Math.sin(p)]);const D=circ(C,dCD,B,l2,Dp);if(!D)return null;Dp=D;const a=ang(sub(D,C));const E=add(C,rot(eL,a));const q=ang(sub(D,B));const G=add(B,rot([l3,0],q+al));out.push({p,q,C,D,E,G,s:len(sub(F,G))})}
return{out,F,C0,D0}}
function evalg(g){const r=sweep(g);if(!r)return null;const o=r.out;const y=o.map(k=>k.E[1]-E0[1]);const ymax=y[y.length-1];if(!(ymax>0))return null;
for(let i=1;i<y.length;i++)if(y[i]<=y[i-1])return null;
const s=o.map(k=>o[0].s-k.s);for(let i=1;i<s.length;i++)if(s[i]<=s[i-1])return null;
// scale to 0.208 travel: find index
return{r,y,s,x:o.map(k=>k.E[0]-E0[0])}}
function metrics(g){const e=evalg(g);if(!e)return null;const{y,s,x}=e;const T=0.208;let k=y.findIndex(v=>v>=T);if(k<0)return null;
const LR=[];for(let i=1;i<=k;i++)LR.push((y[i]-y[i-1])/(s[i]-s[i-1]));
const q=e.r.out;const dq=q[k].q-q[0].q,dp=q[k].p-q[0].p;
return{k,stroke:s[k],LR0:LR[0],LRm:LR[Math.floor(LR.length/2)],LR1:LR[LR.length-1],LR,xs:x.slice(0,k+1),ys:y.slice(0,k+1),counter:Math.sign(dq)!==Math.sign(dp),e}}
function cost(g){const m=metrics(g);if(!m)return 1e9;let c=0;
c+=((m.stroke-0.075)/0.005)**2;
c+=((m.LR0-3.0)/0.1)**2+((m.LR1-2.55)/0.1)**2+((m.LRm-2.78)/0.1)**2;
for(let i=1;i<m.LR.length;i++)if(m.LR[i]>m.LR[i-1]+0.01)c+=5;
if(!m.counter)c+=50;
// S path: rearward (x<0) early, min around 30-50% travel, forward later
const xs=m.xs,n=xs.length;let im=0;for(let i=0;i<n;i++)if(xs[i]<xs[im])im=i;
const xmin=xs[im];c+=((xmin+0.008)/0.003)**2;c+=((im/n-0.5)/0.15)**2;c+=((xs[n-1]-xmin-0.006)/0.004)**2;
const o=m.e.r.out;for(const k of o.slice(0,m.k+1)){if(k.C[1]<0.33)c+=((0.33-k.C[1])/0.01)**2;if(k.C[0]>g.A[0]+0.02)c+=((k.C[0]-g.A[0]-0.02)/0.01)**2;if(k.D[0]>g.B[0]+0.03)c+=((k.D[0]-g.B[0]-0.03)/0.01)**2;}
const F=m.e.r.F;const BB=[0.46,0.40],H1=[1.004,0.891],ST=[0.33,0.80];
const dl=(P,a,b)=>{const d=[b[0]-a[0],b[1]-a[1]],L=Math.hypot(d[0],d[1]);const t=((P[0]-a[0])*d[0]+(P[1]-a[1])*d[1])/L/L;return[Math.abs((P[0]-a[0])*d[1]-(P[1]-a[1])*d[0])/L,t]};
const [dF,tF]=dl(F,BB,H1);if(dF>0.025)c+=((dF-0.025)/0.01)**2;if(tF<0.25||tF>0.8)c+=50;
const [dB,tB]=dl(g.B,BB,ST);if(dB>0.03)c+=((dB-0.03)/0.01)**2;if(tB<0.2||tB>0.7)c+=50;
const [dA]=dl(g.A,BB,ST);if(Math.hypot(g.A[0]-BB[0],g.A[1]-BB[1])>0.07)c+=((Math.hypot(g.A[0]-BB[0],g.A[1]-BB[1])-0.07)/0.01)**2;
for(const k of o.slice(0,m.k+1)){const [dG]=dl(k.G,BB,H1);}
return c}
function rnd(a,b){return a+Math.random()*(b-a)}
function randg(){return{A:[rnd(0.42,0.50),rnd(0.38,0.47)],l1:rnd(0.04,0.09),p0:rnd(-Math.PI,Math.PI),B:[rnd(0.36,0.45),rnd(0.52,0.70)],l2:rnd(0.05,0.12),q0:rnd(-Math.PI,Math.PI),dp:rnd(-2,2),l3:rnd(0.03,0.12),al:rnd(-1.5,1.5),beta:rnd(-0.2,1.3),sl:0.25}}
function mut(g,s){const h=JSON.parse(JSON.stringify(g));h.A[0]+=rnd(-s,s)*0.02;h.A[1]+=rnd(-s,s)*0.02;h.B[0]+=rnd(-s,s)*0.02;h.B[1]+=rnd(-s,s)*0.02;h.l1=Math.min(0.09,Math.max(0.03,h.l1+rnd(-s,s)*0.01));h.A[1]=Math.max(0.37,Math.min(0.48,h.A[1]));h.A[0]=Math.max(0.40,Math.min(0.52,h.A[0]));h.l2=Math.min(0.13,Math.max(0.04,h.l2+rnd(-s,s)*0.01));h.p0+=rnd(-s,s)*0.2;h.q0+=rnd(-s,s)*0.2;h.dp+=rnd(-s,s)*0.1;h.l3=Math.max(0.02,h.l3+rnd(-s,s)*0.01);h.al+=rnd(-s,s)*0.2;h.beta+=rnd(-s,s)*0.1;return h}
let best=null,bc=1e18;
for(let t=0;t<60;t++){let g=randg(),c=cost(g),tries=0;while(c>=1e9&&tries<3000){g=randg();c=cost(g);tries++}if(c>=1e9)continue;
for(let i=0;i<4000;i++){const s=i<2000?1:0.3;const h=mut(g,s),ch=cost(h);if(ch<c){g=h;c=ch}}
if(c<bc){bc=c;best=g}}
console.log('cost',bc);const m=metrics(best);console.log(JSON.stringify(best));
console.log('stroke',m.stroke.toFixed(4),'LR',m.LR0.toFixed(2),m.LRm.toFixed(2),m.LR1.toFixed(2),'counter',m.counter);
for(let i=0;i<m.xs.length;i+=Math.ceil(m.xs.length/10))console.log('y',(m.ys[i]*1000).toFixed(0),'x',(m.xs[i]*1000).toFixed(1),'LR',(m.LR[Math.max(0,i-1)]).toFixed(2));
const o=m.e.r;console.log('C0',o.C0.map(v=>v.toFixed(3)),'D0',o.D0.map(v=>v.toFixed(3)),'F',o.F.map(v=>v.toFixed(3)),'G0',o.out[0].G.map(v=>v.toFixed(3)));
require('fs').writeFileSync('vpp-best.json',JSON.stringify(best));
