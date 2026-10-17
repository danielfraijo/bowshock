import math,sys
def tm(M,beta,g,dth=0.001*math.pi/180):
    ainf2=1/M**2; a02=ainf2+(g-1)/2
    Mn=M*math.sin(beta); r=((g+1)*Mn**2)/((g-1)*Mn**2+2)
    Vn=math.sin(beta)/r; Vt=math.cos(beta)
    vr=Vt; vth=-Vn; th=beta
    V2shock=Vn*Vn+Vt*Vt
    def f(th,vr,vth):
        a2=a02-(g-1)/2*(vr*vr+vth*vth)
        return vth,(vth*vth*vr-a2*(2*vr+vth/math.tan(th)))/(a2-vth*vth)
    while th>1e-4:
        k1=f(th,vr,vth);k2=f(th-dth/2,vr-dth/2*k1[0],vth-dth/2*k1[1])
        k3=f(th-dth/2,vr-dth/2*k2[0],vth-dth/2*k2[1]);k4=f(th-dth,vr-dth*k3[0],vth-dth*k3[1])
        nvr=vr-dth/6*(k1[0]+2*k2[0]+2*k3[0]+k4[0]);nvth=vth-dth/6*(k1[1]+2*k2[1]+2*k3[1]+k4[1])
        if nvth>=0:
            fr=vth/(vth-nvth);return th-fr*dth,vr+(nvr-vr)*fr,V2shock
        th-=dth;vr=nvr;vth=nvth
    return None
def cone_beta(M,cone,g):
    lo=math.asin(1/M)*(1+1e-9);hi=math.radians(70)
    for _ in range(60):
        mid=(lo+hi)/2;r=tm(M,mid,g)
        if r is None: lo=mid;continue
        if r[0]<cone: lo=mid
        else: hi=mid
    return (lo+hi)/2
def cp_cone(M,cone,g=1.4):
    b=cone_beta(M,cone,g);thc,vrc,V2s=tm(M,b,g)
    a02=1/M**2+(g-1)/2;Mn=M*math.sin(b);p21=1+2*g/(g+1)*(Mn**2-1)
    T2=1-(g-1)/2*V2s/a02;Tc=1-(g-1)/2*vrc**2/a02
    pc=p21*(Tc/T2)**(g/(g-1));return b,(pc-1)/(0.5*g*M*M),pc
if __name__=="__main__":
    for M,c in [(8,10),(8,5),(8,20),(5,10),(2,10),(10,15),(3,20),(8,1),(8,2),(8,30)]:
        b,cp,pc=cp_cone(M,math.radians(c))
        print(f"IND M={M} cone={c} beta={math.degrees(b):.5f} Cp={cp:.7f} pc/pinf={pc:.5f}")
