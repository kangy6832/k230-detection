# 双目相机黑色正方形检测系统 — K230 CanMV
# 功能：双目采集 + 立体匹配 + 正方形检测 + PnP 位姿求解 + 3D 坐标输出
# 运行方式：在 CanMV IDE 中打开并运行此脚本

import time
import os
import math

from media.sensor import *
from media.display import *
from media.media import *

# ===================== 配置参数 =====================
LEFT_SENSOR_ID = 0
RIGHT_SENSOR_ID = 1
CAPTURE_WIDTH = 320
CAPTURE_HEIGHT = 240
PIXEL_FORMAT = Sensor.GRAYSCALE
LEFT_H_MIRROR = False
LEFT_V_FLIP = False
RIGHT_H_MIRROR = False
RIGHT_V_FLIP = False

LEFT_CAMERA_MATRIX = [[290.0,0.0,160.0],[0.0,290.0,120.0],[0.0,0.0,1.0]]
RIGHT_CAMERA_MATRIX = [[290.0,0.0,160.0],[0.0,290.0,120.0],[0.0,0.0,1.0]]
LEFT_DIST_COEFFS = [0.0,0.0,0.0,0.0,0.0]
RIGHT_DIST_COEFFS = [0.0,0.0,0.0,0.0,0.0]

BASELINE_MM = 75.0
STEREO_R = [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]
STEREO_T = [BASELINE_MM,0.0,0.0]

STEREO_BLOCK_SIZE = 9
STEREO_MIN_DISP = 0
STEREO_MAX_DISP = 64

SQUARE_SIZE_MM = 100.0
LINE_WIDTH_MM = 3.0
RECT_THRESHOLD = 7000
MIN_RECT_AREA_PX = 400
MAX_RECT_AREA_PX = 60000
MAX_ASPECT_RATIO_DEVIATION = 0.25
MAX_SIDE_RELATIVE_ERROR = 0.18

MAX_REPROJECTION_ERROR_PX = 3.5
MIN_Z_MM = 80.0
MAX_Z_MM = 300.0

ENABLE_SMOOTHING = True
EMA_ALPHA = 0.45
PRINT_EVERY_N_FRAMES = 5
DRAW_DEBUG_OVERLAY = True
USE_HDMI = False

# ===================== 几何工具 =====================
def _dot3(a,b): return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
def _norm3(v): return math.sqrt(_dot3(v,v))
def _cross3(a,b): return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
def _sub3(a,b): return [a[0]-b[0],a[1]-b[1],a[2]-b[2]]
def _add3(a,b): return [a[0]+b[0],a[1]+b[1],a[2]+b[2]]
def _mul3(v,s): return [v[0]*s,v[1]*s,v[2]*s]
def _mat3_mul_vec(m,v): return [m[0][0]*v[0]+m[0][1]*v[1]+m[0][2]*v[2],m[1][0]*v[0]+m[1][1]*v[1]+m[1][2]*v[2],m[2][0]*v[0]+m[2][1]*v[1]+m[2][2]*v[2]]
def _mat3_inv(m):
    a,b,c=m[0];d,e,f=m[1];g,h,i=m[2]
    A=e*i-f*h;B=-(d*i-f*g);C=d*h-e*g;D=-(b*i-c*h);E=a*i-c*g;F=-(a*h-b*g);G=b*f-c*e;H=-(a*f-c*d);I=a*e-b*d
    det=a*A+b*B+c*C
    if abs(det)<1e-9: return None
    inv_det=1.0/det
    return [[A*inv_det,D*inv_det,G*inv_det],[B*inv_det,E*inv_det,H*inv_det],[C*inv_det,F*inv_det,I*inv_det]]
def _distance2(p0,p1):
    dx=p0[0]-p1[0];dy=p0[1]-p1[1];return math.sqrt(dx*dx+dy*dy)
def _distance3(p0,p1):
    dx=p0[0]-p1[0];dy=p0[1]-p1[1];dz=p0[2]-p1[2];return math.sqrt(dx*dx+dy*dy+dz*dz)
def _angle_between(v1,v2):
    d1=_norm3(v1);d2=_norm3(v2)
    if d1<1e-9 or d2<1e-9: return 0.0
    cos_val=max(-1.0,min(1.0,_dot3(v1,v2)/(d1*d2)))
    return math.degrees(math.acos(cos_val))

def order_corners_tl_tr_br_bl(points):
    pts=[(float(p[0]),float(p[1])) for p in points]
    if len(pts)!=4: return None
    sums=[p[0]+p[1] for p in pts];diffs=[p[0]-p[1] for p in pts]
    tl=pts[sums.index(min(sums))];br=pts[sums.index(max(sums))]
    tr=pts[diffs.index(max(diffs))];bl=pts[diffs.index(min(diffs))]
    ordered=[tl,tr,br,bl]
    return ordered if len(set(ordered))==4 else None

def is_square_like(ordered_points,max_aspect,max_side_err):
    if ordered_points is None: return False
    tl,tr,br,bl=ordered_points
    top=_distance2(tl,tr);right=_distance2(tr,br);bottom=_distance2(br,bl);left=_distance2(bl,tl)
    if min(top,right,bottom,left)<2.0: return False
    mean_side=(top+right+bottom+left)/4.0
    side_err=max(abs(top-mean_side),abs(right-mean_side),abs(bottom-mean_side),abs(left-mean_side))/mean_side
    if side_err>max_side_err: return False
    width=(top+bottom)*0.5;height=(left+right)*0.5
    aspect=width/height if height>1e-6 else 999.0
    if abs(aspect-1.0)>max_aspect: return False
    for prev_pt,vertex,next_pt in [(tl,tr,br),(tr,br,bl),(br,bl,tl),(bl,tl,tr)]:
        v1=[prev_pt[0]-vertex[0],prev_pt[1]-vertex[1],0.0]
        v2=[next_pt[0]-vertex[0],next_pt[1]-vertex[1],0.0]
        if _angle_between(v1,v2)<80 or _angle_between(v1,v2)>100: return False
    diag1=_distance2(tl,br);diag2=_distance2(tr,bl)
    if abs(diag1-diag2)/max(diag1,diag2,1e-6)>0.10: return False
    return True

def solve_linear_system(A,b):
    n=len(b);aug=[A[i][:]+[b[i]] for i in range(n)]
    for col in range(n):
        pivot=col;max_abs=abs(aug[col][col])
        for row in range(col+1,n):
            if abs(aug[row][col])>max_abs: max_abs=abs(aug[row][col]);pivot=row
        if max_abs<1e-9: return None
        if pivot!=col: aug[col],aug[pivot]=aug[pivot],aug[col]
        pv=aug[col][col]
        for j in range(col,n+1): aug[col][j]/=pv
        for row in range(n):
            if row==col: continue
            f=aug[row][col]
            if abs(f)<1e-12: continue
            for j in range(col,n+1): aug[row][j]-=f*aug[col][j]
    return [aug[i][n] for i in range(n)]

def solve_homography(world_xy,image_uv):
    if len(world_xy)!=4 or len(image_uv)!=4: return None
    A=[];b=[]
    for (xw,yw),(u,v) in zip(world_xy,image_uv):
        A.append([xw,yw,1.0,0.0,0.0,0.0,-u*xw,-u*yw]);b.append(u)
        A.append([0.0,0.0,0.0,xw,yw,1.0,-v*xw,-v*yw]);b.append(v)
    h=solve_linear_system(A,b)
    if h is None: return None
    return [[h[0],h[1],h[2]],[h[3],h[4],h[5]],[h[6],h[7],1.0]]

def solve_pose_ippe(image_points,camera_matrix,square_size_mm):
    half=square_size_mm*0.5
    world_xy=[(-half,-half),(half,-half),(half,half),(-half,half)]
    H=solve_homography(world_xy,image_points)
    if H is None: return None,None,None
    K_inv=_mat3_inv(camera_matrix)
    if K_inv is None: return None,None,None
    b1=_mat3_mul_vec(K_inv,[H[0][0],H[1][0],H[2][0]])
    b2=_mat3_mul_vec(K_inv,[H[0][1],H[1][1],H[2][1]])
    b3=_mat3_mul_vec(K_inv,[H[0][2],H[1][2],H[2][2]])
    n1=_norm3(b1);n2=_norm3(b2)
    if n1<1e-9 or n2<1e-9: return None,None,None
    scale=2.0/(n1+n2)
    r1=_mul3(b1,scale);r2=_mul3(b2,scale);t=_mul3(b3,scale)
    r1n=_mul3(r1,1.0/_norm3(r1))
    if _norm3(r1n)<1e-9: return None,None,None
    r2_ortho=[r2[i]-_dot3(r2,r1n)*r1n[i] for i in range(3)]
    r2n=_mul3(r2_ortho,1.0/_norm3(r2_ortho))
    if _norm3(r2n)<1e-9: return None,None,None
    r3n=_cross3(r1n,r2n)
    if _norm3(r3n)<1e-9: return None,None,None
    r3n=_mul3(r3n,1.0/_norm3(r3n))
    if t[2]<0: r1n=_mul3(r1n,-1.0);r2n=_mul3(r2n,-1.0);r3n=_mul3(r3n,-1.0);t=_mul3(t,-1.0)
    R=[[r1n[0],r2n[0],r3n[0]],[r1n[1],r2n[1],r3n[1]],[r1n[2],r2n[2],r3n[2]]]
    return R,t,compute_reprojection_error(image_points,R,t,camera_matrix,square_size_mm)

def project_world_point(world_xyz,R,t,camera_matrix):
    xc=R[0][0]*world_xyz[0]+R[0][1]*world_xyz[1]+R[0][2]*world_xyz[2]+t[0]
    yc=R[1][0]*world_xyz[0]+R[1][1]*world_xyz[1]+R[1][2]*world_xyz[2]+t[1]
    zc=R[2][0]*world_xyz[0]+R[2][1]*world_xyz[1]+R[2][2]*world_xyz[2]+t[2]
    if abs(zc)<1e-9: return None
    fx=camera_matrix[0][0];fy=camera_matrix[1][1];cx=camera_matrix[0][2];cy=camera_matrix[1][2]
    return (fx*(xc/zc)+cx,fy*(yc/zc)+cy)

def compute_reprojection_error(image_points,R,t,camera_matrix,square_size_mm):
    half=square_size_mm*0.5
    wps=[(-half,-half,0.0),(half,-half,0.0),(half,half,0.0),(-half,half,0.0)]
    total=0.0;count=0
    for i,wp in enumerate(wps):
        uv=project_world_point(wp,R,t,camera_matrix)
        if uv is None: continue
        du=uv[0]-image_points[i][0];dv=uv[1]-image_points[i][1]
        total+=math.sqrt(du*du+dv*dv);count+=1
    return total/count if count>0 else 1e9

def square_corners_camera_coords(R,t,square_size_mm):
    half=square_size_mm*0.5
    wps=[(-half,-half,0.0),(half,-half,0.0),(half,half,0.0),(-half,half,0.0)]
    cc=[]
    for wp in wps:
        xc=R[0][0]*wp[0]+R[0][1]*wp[1]+R[0][2]*wp[2]+t[0]
        yc=R[1][0]*wp[0]+R[1][1]*wp[1]+R[1][2]*wp[2]+t[1]
        zc=R[2][0]*wp[0]+R[2][1]*wp[1]+R[2][2]*wp[2]+t[2]
        cc.append((xc,yc,zc))
    return cc

def ema_points(prev,new,alpha):
    if prev is None: return new
    return [tuple(alpha*n+(1.0-alpha)*p for p,n in zip(pv,nv)) for pv,nv in zip(prev,new)]

def pick_best_square(rects):
    best=None;best_score=-1.0
    for r in rects:
        area=float(r.w())*float(r.h())
        if area<MIN_RECT_AREA_PX or area>MAX_RECT_AREA_PX: continue
        try: mag=float(r.magnitude())
        except: mag=0.0
        if mag<1000: continue
        ordered=order_corners_tl_tr_br_bl(r.corners())
        if not is_square_like(ordered,MAX_ASPECT_RATIO_DEVIATION,MAX_SIDE_RELATIVE_ERROR): continue
        score=mag+0.002*area
        if score>best_score: best_score=score;best=(r,ordered,score)
    return best

# ===================== 立体匹配 =====================
def block_match_simple(img_l,img_r,block_size,min_disp,num_disp):
    w=img_l.width();h=img_l.height();half=block_size//2;max_d=min_disp+num_disp
    disparity=[]
    for y in range(half,h-half):
        row=[]
        for x in range(half,w-half):
            best_d=-1;best_c=0x7FFFFFFF
            for d in range(max(min_disp,0),min(max_d,x-half+1)):
                cost=0
                for dy in range(-half,half+1):
                    for dx in range(-half,half+1):
                        cost+=abs(img_l.get_pixel(x+dx,y+dy)-img_r.get_pixel(x+dx-d,y+dy))
                        if cost>=best_c: break
                    if cost>=best_c: break
                if cost<best_c: best_c=cost;best_d=d
            row.append(best_d)
        disparity.append(row)
    return disparity

def disparity_to_depth(dmap,focal,baseline,min_v=1):
    depth=[]
    for row in dmap:
        dr=[]
        for d in row:
            dr.append((focal*baseline)/d if d>=min_v else -1)
        depth.append(dr)
    return depth

def get_region_disparity(dmap,cx,cy,rs):
    half=rs//2;vals=[];h=len(dmap)
    if h==0: return -1
    w=len(dmap[0])
    for y in range(max(0,cy-half),min(h,cy+half)):
        for x in range(max(0,cx-half),min(w,cx-half)):
            d=dmap[y][x]
            if d>0: vals.append(d)
    if not vals: return -1
    vals.sort();return vals[len(vals)//2]

# ===================== 主程序 =====================
def main():
    sensor_l=None;sensor_r=None;smoothed_corners=None;frame_id=0
    try:
        print("init left sensor...")
        sensor_l=Sensor(id=LEFT_SENSOR_ID);sensor_l.reset()
        sensor_l.set_framesize(width=CAPTURE_WIDTH,height=CAPTURE_HEIGHT)
        sensor_l.set_pixformat(PIXEL_FORMAT)
        sensor_l.set_hmirror(LEFT_H_MIRROR);sensor_l.set_vflip(LEFT_V_FLIP)

        print("init right sensor...")
        sensor_r=Sensor(id=RIGHT_SENSOR_ID);sensor_r.reset()
        sensor_r.set_framesize(width=CAPTURE_WIDTH,height=CAPTURE_HEIGHT)
        sensor_r.set_pixformat(PIXEL_FORMAT)
        sensor_r.set_hmirror(RIGHT_H_MIRROR);sensor_r.set_vflip(RIGHT_V_FLIP)

        if USE_HDMI:
            bi=sensor_l.bind_info(x=0,y=0);Display.bind_layer(**bi,layer=Display.LAYER_VIDEO1)
            bi=sensor_r.bind_info(x=CAPTURE_WIDTH,y=0);Display.bind_layer(**bi,layer=Display.LAYER_VIDEO2)
            Display.init(Display.LT9611,to_ide=True)
        else:
            Display.init(Display.VIRT,width=CAPTURE_WIDTH*2,height=CAPTURE_HEIGHT,fps=30,to_ide=True)

        MediaManager.init();sensor_l.run()
        clock=time.clock()
        print("dual stereo detection started")

        while True:
            os.exitpoint();clock.tick();frame_id+=1
            img_l=sensor_l.snapshot();img_r=sensor_r.snapshot()
            rects=img_l.find_rects(threshold=RECT_THRESHOLD)
            best=pick_best_square(rects)
            status="NO_TARGET"

            if best is not None:
                _,ordered,_=best
                R,t,reproj_err=solve_pose_ippe(ordered,LEFT_CAMERA_MATRIX,SQUARE_SIZE_MM)
                if (R is not None and t is not None and reproj_err is not None
                        and reproj_err<=MAX_REPROJECTION_ERROR_PX
                        and MIN_Z_MM<=t[2]<=MAX_Z_MM):
                    corners_cam=square_corners_camera_coords(R,t,SQUARE_SIZE_MM)
                    if ENABLE_SMOOTHING:
                        corners_cam=ema_points(smoothed_corners,corners_cam,EMA_ALPHA)
                    smoothed_corners=corners_cam
                    status="OK"
                    if DRAW_DEBUG_OVERLAY:
                        for i,p in enumerate(ordered):
                            img_l.draw_circle(int(p[0]),int(p[1]),4,color=(0,255,0))
                            img_l.draw_string(int(p[0])+3,int(p[1])+3,str(i),color=(0,255,0))
                        img_l.draw_string(2,2,"OK err=%.2f Z=%.1fmm"%(reproj_err,t[2]),color=(0,255,0))
                        tl,tr,br,bl=corners_cam
                        img_l.draw_string(2,20,"TL(%.1f,%.1f,%.1f)"%(tl[0],tl[1],tl[2]))
                        img_l.draw_string(2,38,"TR(%.1f,%.1f,%.1f)"%(tr[0],tr[1],tr[2]))
                else:
                    smoothed_corners=None;status="UNRELIABLE"
                    if DRAW_DEBUG_OVERLAY:
                        img_l.draw_string(2,2,"UNRELIABLE err=%.2f"%reproj_err,color=(255,80,80))
            else:
                smoothed_corners=None
                if DRAW_DEBUG_OVERLAY:
                    img_l.draw_string(2,2,"NO_TARGET",color=(255,255,0))

            if not USE_HDMI:
                comb=image.Image(CAPTURE_WIDTH*2,CAPTURE_HEIGHT,img_l.format())
                comb.draw_image(img_l,0,0);comb.draw_image(img_r,CAPTURE_WIDTH,0)
                comb.draw_line(CAPTURE_WIDTH,0,CAPTURE_WIDTH,CAPTURE_HEIGHT,color=255,thickness=1)
                Display.show_image(comb)

            if smoothed_corners is not None and frame_id%PRINT_EVERY_N_FRAMES==0:
                tl,tr,br,bl=smoothed_corners
                print("%d,%s,TL(%.1f,%.1f,%.1f),TR(%.1f,%.1f,%.1f),BR(%.1f,%.1f,%.1f),BL(%.1f,%.1f,%.1f)"%(
                    frame_id,status,tl[0],tl[1],tl[2],tr[0],tr[1],tr[2],br[0],br[1],br[2],bl[0],bl[1],bl[2]))
            elif frame_id%PRINT_EVERY_N_FRAMES==0:
                print("%d,%s"%(frame_id,status))

    except KeyboardInterrupt: print("user stop")
    except Exception as e:
        print("error:",e);import sys;sys.print_exception(e)
    finally:
        if sensor_l: sensor_l.stop()
        if sensor_r: sensor_r.stop()
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP);time.sleep_ms(100)
        MediaManager.deinit()
        print("stopped")

main()
