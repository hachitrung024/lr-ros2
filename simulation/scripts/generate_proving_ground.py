#!/usr/bin/env python3
"""Generate deterministic rollover / contact proving ground for V2."""
import math
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

def box(name,x,y,z,sx,sy,sz,roll=0,pitch=0,yaw=0,color="0.35 0.35 0.35 1",mu=0.9,mu2=0.7):
    return f'''<model name="{name}"><static>true</static><pose>{x} {y} {z} {roll} {pitch} {yaw}</pose><link name="link">
    <collision name="collision"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry><surface><friction><ode><mu>{mu}</mu><mu2>{mu2}</mu2></ode></friction></surface></collision>
    <visual name="visual"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual></link></model>'''

def mound(name,x,y,ramp=6,top=4,width=5,angle_deg=22,yaw=0):
    a=math.radians(angle_deg); t=.18; h=ramp*math.sin(a); off=top/2+ramp/2; ux,uy=math.cos(yaw),math.sin(yaw)
    z_r=(ramp*math.sin(a)-t*math.cos(a))/2
    return [box(name+'_up',x-off*ux,y-off*uy,z_r,ramp,width,t,pitch=-a,yaw=yaw),
            box(name+'_top',x,y,h-t/2,top,width,t,yaw=yaw),
            box(name+'_down',x+off*ux,y+off*uy,z_r,ramp,width,t,pitch=a,yaw=yaw)]

def vegetation(name, x, y, scale=1.0, yaw=0):
    """Camera-visible vegetation with deliberately no collision geometry."""
    blades=[]
    offsets=[(-.25,-.12,.65,.035),(-.08,.18,.90,-.05),(.10,-.18,.75,.07),
             (.24,.10,1.05,-.03),(0,0,.82,.02)]
    for i,(ox,oy,height,lean) in enumerate(offsets):
        h=height*scale
        blades.append(f'''<visual name="blade_{i}"><pose>{ox*scale} {oy*scale} {h/2} 0 {lean} {i*.73}</pose>
        <geometry><box><size>{.035*scale} {.16*scale} {h}</size></box></geometry>
        <material><ambient>.12 .38 .07 1</ambient><diffuse>.18 .52 .10 1</diffuse></material></visual>''')
    # A low bush core makes the patch clearly visible to RGB/depth cameras.
    blades.append(f'''<visual name="bush_core"><pose>0 0 {.28*scale} 0 0 0</pose>
      <geometry><sphere><radius>{.34*scale}</radius></sphere></geometry>
      <material><ambient>.10 .30 .06 1</ambient><diffuse>.15 .43 .08 1</diffuse></material></visual>''')
    return f'''<model name="{name}"><static>true</static><pose>{x} {y} 0 0 0 {yaw}</pose>
      <link name="vegetation_link">{''.join(blades)}</link></model>'''

def main():
    pieces=[]
    pieces += mound('mound_18deg',-10,-15,angle_deg=18,width=5)
    pieces += mound('mound_28deg',10,-15,angle_deg=28,width=6)
    # Long side-slope face for sustained lateral load transfer.
    a=math.radians(32); length=8; t=.20; z=(length*math.sin(a)-t*math.cos(a))/2
    pieces.append(box('long_cross_slope',-14,8,z,length,20,t,pitch=-a,color="0.42 0.34 0.25 1"))
    # Asymmetric one-track bumps: excite roll without pitching both tracks together.
    for i,x in enumerate(range(-2,19,3)):
        side=1 if i%2==0 else -1
        pieces.append(box(f'one_track_bump_{i}',x,8+side*.43,.10,1.0,.34,.20,pitch=(-1 if i%2 else 1)*.12,color="0.45 0.28 0.16 1"))
    # Rough corridor: deterministic height / yaw changes for repeatable regression.
    heights=[.08,.16,.11,.22,.07,.18,.13,.25]
    for i,h in enumerate(heights):
        pieces.append(box(f'rough_patch_{i}',-18+i*2.4,22,h/2,2.0,3.2,h,yaw=(-.08 if i%2 else .08),color="0.30 0.24 0.16 1"))
    # Split-friction lane; thin plates create left/right traction imbalance.
    pieces.append(box('low_friction_left',8,21,.006,12,2.0,.012,color="0.18 0.28 0.48 1",mu=.20,mu2=.15))
    pieces.append(box('high_friction_right',8,17,.006,12,2.0,.012,color="0.48 0.28 0.18 1",mu=1.20,mu2=.90))
    # Fixed collision objects.
    pieces += [box('concrete_block',24,-2,.45,1.6,1.0,.9,yaw=.3,color="0.55 0.18 0.08 1"),
               box('offset_block',20,10,.30,1.0,.8,.6,yaw=-.4,color="0.55 0.18 0.08 1")]
    # Traversable grass / bushes. Visual-only geometry lets the rover pass through
    # while ZED RGB and depth streams still observe vegetation-like clutter.
    vegetation_layout=[
        (-23,-19,.8,.2),(-19,-20,1.1,-.4),(-15,-18,.7,.8),
        (-5,-11,1.0,.1),(0,-12,.8,-.6),(16,-11,1.2,.5),
        (22,-7,.9,-.2),(25,4,1.1,.7),(17,13,.7,.3),
        (3,14,1.0,-.8),(-5,16,.9,.4),(-22,16,1.15,-.1),
        (-26,4,.8,.6),(-7,3,.75,-.5),(10,4,1.0,.25),
    ]
    for i,(x,y,scale,yaw) in enumerate(vegetation_layout):
        pieces.append(vegetation(f'traversable_vegetation_{i}',x,y,scale,yaw))
    world=f'''<?xml version="1.0"?><sdf version="1.9"><world name="rollover_v2">
    <physics name="physics" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics><gravity>0 0 -9.81</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><direction>-.45 .25 -1</direction></light>
    <model name="ground"><static>true</static><link name="link"><collision name="collision"><geometry><plane><normal>0 0 1</normal><size>80 60</size></plane></geometry><surface><friction><ode><mu>.85</mu><mu2>.65</mu2></ode></friction></surface></collision><visual name="visual"><geometry><plane><normal>0 0 1</normal><size>80 60</size></plane></geometry><material><ambient>.25 .38 .20 1</ambient><diffuse>.25 .38 .20 1</diffuse></material></visual></link></model>
    {''.join(pieces)}
    <include><uri>model://rover_v2</uri><name>rover_v2</name><pose>-28 -22 .8 0 0 0</pose></include>
    </world></sdf>'''
    (ROOT/'worlds/rollover_proving_ground.sdf').write_text(world)
    print(f'Generated {ROOT}/worlds/rollover_proving_ground.sdf with {len(pieces)} terrain/object models')

if __name__=='__main__':main()
