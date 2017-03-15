# cmd: RUNRIPPLE (gait 4+2), RUNTRIPOD (gait 3+3), STOP, PAUSE
# fi: the angle of foothold
# velocity: velocity of the movement
# alpha: the direction
# scale: the scale of the stride of the foot
int32 cmd
float64 fi
float64 velocity
float64 alpha
float64 scale

int32 RUNRIPPLE=1
int32 RUNTRIPOD=2
int32 STOP=3
int32 PAUSE=4
int32 RUNTWAVE=5
int32 TOINIT = 6
