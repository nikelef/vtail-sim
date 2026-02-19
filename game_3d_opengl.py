import math
import sys
import pygame
from pygame.locals import DOUBLEBUF, OPENGL

from OpenGL.GL import *
from OpenGL.GLU import *

from core_sim import SimParams, reset_state, step_sim, clamp

W, H = 1280, 720
FPS = 60

def draw_grid(size=5000, step=250):
    glColor3f(0.85, 0.85, 0.85)
    glBegin(GL_LINES)
    for x in range(-size, size + 1, step):
        glVertex3f(x, 0, -size)
        glVertex3f(x, 0, size)
    for z in range(-size, size + 1, step):
        glVertex3f(-size, 0, z)
        glVertex3f(size, 0, z)
    glEnd()

def draw_target_ring(px, pz, r):
    glColor3f(0.1, 0.6, 0.1)
    glBegin(GL_LINE_LOOP)
    for i in range(72):
        a = 2 * math.pi * i / 72
        glVertex3f(px + r * math.cos(a), 0.02, pz + r * math.sin(a))
    glEnd()
    glPointSize(6.0)
    glBegin(GL_POINTS)
    glVertex3f(px, 0.03, pz)
    glEnd()

def draw_trail(hist):
    if len(hist) < 2:
        return
    glColor3f(0.2, 0.2, 0.7)
    glBegin(GL_LINE_STRIP)
    for row in hist:
        # Map y->z for OpenGL ground plane
        glVertex3f(row["x_m"], 0.05, row["y_m"])
    glEnd()

def draw_vtail_aircraft(x, alt, y, heading_rad, bank_deg, left_rv, right_rv):
    # Simple body in local coordinates
    glPushMatrix()
    glTranslatef(x, alt, y)
    glRotatef(-math.degrees(heading_rad), 0, 1, 0)  # yaw about Y
    glRotatef(bank_deg, 0, 0, 1)  # roll about Z (visual proxy)

    # Body
    glColor3f(0.05, 0.05, 0.05)
    glBegin(GL_LINES)
    # fuselage
    glVertex3f(-6, 0, 0)
    glVertex3f(10, 0, 0)
    # wings
    glVertex3f(2, 0, -10)
    glVertex3f(2, 0, 10)
    glEnd()

    # V-tail surfaces (two lines). Deflection displayed by rotating around local x a bit.
    # This is only a visualization of left/right ruddervator deflection.
    defl_scale = 18.0  # degrees visual
    glPushMatrix()
    glTranslatef(-6, 0, 0)
    glColor3f(0.1, 0.1, 0.1)

    # Left surface
    glPushMatrix()
    glRotatef(defl_scale * left_rv, 1, 0, 0)
    glBegin(GL_LINES)
    glVertex3f(0, 0, 0)
    glVertex3f(-5, 3, -4)
    glEnd()
    glPopMatrix()

    # Right surface
    glPushMatrix()
    glRotatef(defl_scale * right_rv, 1, 0, 0)
    glBegin(GL_LINES)
    glVertex3f(0, 0, 0)
    glVertex3f(-5, 3, 4)
    glEnd()
    glPopMatrix()

    glPopMatrix()
    glPopMatrix()

def set_camera_chase(x, alt, y, heading_rad):
    # Chase camera behind and above aircraft
    back = 55.0
    up = 22.0
    side = 12.0

    hx = math.cos(heading_rad)
    hz = math.sin(heading_rad)

    cam_x = x - hx * back - hz * side
    cam_y = alt + up
    cam_z = y - hz * back + hx * side

    look_x = x + hx * 30.0
    look_y = alt + 6.0
    look_z = y + hz * 30.0

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt(cam_x, cam_y, cam_z, look_x, look_y, look_z, 0, 1, 0)

def draw_hud(screen, p, s):
    # Simple HUD overlay via pygame
    # (We draw after swapping buffers, using a separate 2D surface blit)
    pass

def main():
    pygame.init()
    pygame.display.set_caption("V-Tail Drop Glide — 3D (Energy + Turn Sink + Ruddervator Mixer)")
    pygame.display.set_mode((W, H), DOUBLEBUF | OPENGL)

    glEnable(GL_DEPTH_TEST)
    glClearColor(0.96, 0.96, 0.96, 1.0)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(60, W / H, 0.1, 20000.0)

    p = SimParams()
    s = reset_state(p)

    clock = pygame.time.Clock()

    while True:
        dt = clock.tick(FPS) / 1000.0

        pitch_in = 0.0
        yaw_in = 0.0

        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                pygame.quit()
                sys.exit(0)

        keys = pygame.key.get_pressed()
        if keys[pygame.K_ESCAPE]:
            pygame.quit()
            sys.exit(0)

        if keys[pygame.K_w]:
            pitch_in += 1.0
        if keys[pygame.K_s]:
            pitch_in -= 1.0
        if keys[pygame.K_a]:
            yaw_in += 1.0
        if keys[pygame.K_d]:
            yaw_in -= 1.0

        yaw_in = clamp(yaw_in, -1.0, 1.0)
        pitch_in = clamp(pitch_in, -1.0, 1.0)

        if keys[pygame.K_r]:
            s = reset_state(p)

        # Step simulation
        step_sim(p, s, dt, pitch_in, yaw_in)

        # Render
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        set_camera_chase(s.x_m, s.alt_m, s.y_m, s.heading_rad)

        # Ground
        glColor3f(0.90, 0.90, 0.90)
        glBegin(GL_QUADS)
        size = 8000
        glVertex3f(-size, 0, -size)
        glVertex3f(size, 0, -size)
        glVertex3f(size, 0, size)
        glVertex3f(-size, 0, size)
        glEnd()

        draw_grid(size=6000, step=300)
        draw_target_ring(p.target_x_m, p.target_y_m, p.target_radius_m)
        draw_trail(s.hist)

        draw_vtail_aircraft(
            s.x_m, s.alt_m, s.y_m,
            s.heading_rad, s.bank_deg,
            s.left_rv, s.right_rv
        )

        pygame.display.flip()

        # Print impact once
        if (not s.alive) and s.impact is not None:
            # prevent repeated prints by clearing after first print
            print(f"IMPACT at t={s.impact['t_s']:.1f}s  miss={s.impact['miss_m']:.1f} m  "
                  f"{'ON TARGET' if s.impact['on_target'] else 'MISS'}")
            s.impact = None

if __name__ == "__main__":
    main()
