import time

import pygame
import sys
#print(sys.argv[1])
# initializing the constructor
sport = '/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_002900343532511431333430-if02'
pygame.init()
# screen resolution
res = (1300, 696)
# opens up a window
screen = pygame.display.set_mode(res)
pygame.display.set_caption('Home')
#-------------------------load assets----------------------
programIcon = pygame.image.load('./assets/icon.png').convert_alpha()
pygame.display.set_icon(programIcon)
background = pygame.image.load('./assets/home/Home.png').convert_alpha()

#reset button
home_unclick = pygame.image.load('./assets/home/home_unclick.png').convert_alpha()
home_click = pygame.image.load('./assets/home/home_click.png').convert_alpha()


#assessment button
assessment_unclick = pygame.image.load('./assets/home/assessment_unclick.png').convert_alpha()
assessment_click = pygame.image.load('./assets/home/assessment_click.png').convert_alpha()

#active traj button
active_traj_unclick = pygame.image.load('./assets/home/active_traj_unclick.png').convert_alpha()
active_traj_click = pygame.image.load('./assets/home/active_traj_click.png').convert_alpha()

#passive traj button
passive_traj_unclick = pygame.image.load('./assets/home/passive_traj_unclick.png').convert_alpha()
passive_traj_click = pygame.image.load('./assets/home/passive_traj_click.png').convert_alpha()

#game button
game_unclick = pygame.image.load('./assets/home/game_unclick.png').convert_alpha()
game_click = pygame.image.load('./assets/home/game_click.png').convert_alpha()

w = 280
h = 260

clock = pygame.time.Clock()
finish = False

#reset font
smallfont = pygame.font.SysFont('Corbel', 70,bold=True )
wait = smallfont.render('Resetting...', True, (0, 0, 0))


while True:

    screen.blit(background, (0, 0))
    pygame.display.set_caption('Home')
    for ev in pygame.event.get():
        if ev.type == pygame.QUIT:
            finish = True

        if ev.type == pygame.MOUSEBUTTONDOWN:

            if 20 <= mouse[0] <= 20+w and 120 <= mouse[1] <= 120+h:
                print("resetting")
                screen.blit(wait, (400, 350))
                pygame.display.update()
                exec(open("homing.py").read(),{'sport':sport})
                #time.sleep(1)
                pygame.event.clear()

            elif 320 <= mouse[0] <= 320+w and 120 <= mouse[1] <= 120+h:
                print("assessment mode")
                exec(open("transparency_tau_w_6doft_test.py").read(),{'sport':sport})
                pygame.event.clear()


            elif 620 <= mouse[0] <= 620+w and 120 <= mouse[1] <= 120+h:
                exec(open("circle_fixed.py").read(),{'sport':sport})
                pygame.event.clear()
                print("traj mode")
            elif 20 <= mouse[0] <= 20+w and 400 <= mouse[1] <= 400+h:
                # exec(open("circle.py").read(),{'sport':sport})
                pygame.event.clear()
                print("assist mode, TBD")
            elif 320 <= mouse[0] <= 320+w and 400 <= mouse[1] <= 400+h:
                exec(open("PyGalaxian.py").read(),{'sport':sport})
                pygame.event.clear()
                screen = pygame.display.set_mode(res)
                print("game mode")

    if not finish:
        # fills the screen with a color
        mouse = pygame.mouse.get_pos()

        #1 homing button/reset
        if 20 <= mouse[0] <= 20+w and 120 <= mouse[1] <= 120+h:
            screen.blit(home_click, (20,120))
        else:
            screen.blit(home_unclick, (20, 120))


        # 2 free button/assessment
        if 320 <= mouse[0] <= 320+w and 120 <= mouse[1] <= 120+h:
            screen.blit(assessment_click, (320,120))
        else:
            screen.blit(assessment_unclick, (320,120))


        # 3 traj button/active traj
        if 620 <= mouse[0] <= 620+w and 120 <= mouse[1] <= 120+h:
            screen.blit(active_traj_click, (620,120))
        else:
            screen.blit(active_traj_unclick, (620, 120))


        # 4 assit button/passive traj
        if 20 <= mouse[0] <= 20+w and 400 <= mouse[1] <= 400+h:
            screen.blit(passive_traj_click, (20,400))
        else:
            screen.blit(passive_traj_unclick, (20,400))

        # 5 game button
        if 320 <= mouse[0] <= 320+w and 400 <= mouse[1] <= 400+h:
            screen.blit(game_click, (320, 400))
        else:
            screen.blit(game_unclick, (320, 400))



        # updates the frames of the game
        pygame.display.update()
        clock.tick(60)
    else:
        pygame.quit()
        break