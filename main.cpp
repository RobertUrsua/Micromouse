#include "mbed.h"
#include "Motor.h"
#include "QEI.h"
#include "EncoderPair.h"
#include "globalConstants.h"
#include "BasicController.h"
#include "CellNavigator.h"


int SOLVEMODE = 0;
// 0 if facing up
// 1 if facing right
DigitalOut myLED(LED1);


// MOTORS
Motor motorl(PB_10, PC_7, MOTOR_PWM_PERIOD, MAX_PWM_PULSELENGTH);
Motor motorr(PA_7, PB_6, MOTOR_PWM_PERIOD, MAX_PWM_PULSELENGTH);


// SENSORS
IRSensor irll(PB_7, PC_0);
IRSensor irfl(PB_0, PC_1);
IRSensor irfr(PC_11, PA_4);
IRSensor irrr(PC_10, PA_0);

// SERIAL
Serial pc(PA_9, PA_10);

// ENCODERS
QEI encl (PA_1, PC_4, NC, 1212, QEI::X4_ENCODING);
QEI encr (PB_3, PA_15, NC, 1212, QEI::X4_ENCODING);

// BASIC CONTROLLER
CellNavigator robot(&motorr,&motorl);


void turnDirRight(int &dir)
{
    switch(dir)
    {
        case 0:
            dir = 1;
            break;
        case 1:
            dir = 2;
            break;
        case 2:
            dir = 3;
            break;
        case 3:
            dir = 0;
            break;
    }
}


void turnDirLeft(int &dir)
{
    switch(dir)
    {
        case 0:
            dir = 3;
            break;
        case 1:
            dir = 0;
            break;
        case 2:
            dir = 1;
            break;
        case 3:
            dir = 2;
            break;
    }
}

struct Cell{
    int x;
    int y;
    int d;
};

void calcDists(int dists[][MAZE_SIZE_Y], char walls[][MAZE_SIZE_Y+1], Cell target){
    int visited[MAZE_SIZE_X][MAZE_SIZE_Y] = {};
    Cell queue[1000];
    unsigned length = 1;
    unsigned front = 0;
    queue[0] = target;
    visited[target.x][target.y] = 1;
    while(front<length){
        Cell cur = queue[front];
        //pc.printf("Processing %d %d\r\n", queue[front].x,queue[front].y);
        // PUT CELL ABOVE
        dists[cur.x][cur.y] = cur.d;
        if((cur.y+1)<MAZE_SIZE_Y){
            if(  (walls[cur.x][cur.y+1]&1) == 0   &&   visited[cur.x][cur.y+1]==0)
            {
                queue[length].x = cur.x;
                queue[length].y = cur.y + 1;
                queue[length].d = cur.d + 1;
                visited[cur.x][cur.y+1] = 1;
                length++;
            }   
        }
        // PUT CELL BELOW
        if((cur.y)>0){
            if(  (walls[cur.x][cur.y]&1) == 0   &&   visited[cur.x][cur.y-1]==0)
            {
                queue[length].x = cur.x;
                queue[length].y = cur.y - 1;
                queue[length].d = cur.d + 1;
                visited[cur.x][cur.y-1] = 1;
                length++;
            }   
        }
        // PUT CELL TO RIGHT
        if((cur.x+1)<MAZE_SIZE_X){
            if(  (walls[cur.x+1][cur.y]&2) == 0   &&   visited[cur.x+1][cur.y]==0)
            {
                queue[length].x = cur.x + 1;
                queue[length].y = cur.y;
                queue[length].d = cur.d + 1;
                visited[cur.x+1][cur.y] = 1;
                length++;
            }   
        }
        // PUT CELL TO LEFT
        if((cur.x)>0){
            if(  (walls[cur.x][cur.y]&2) == 0   &&   visited[cur.x-1][cur.y]==0)
            {
                queue[length].x = cur.x - 1;
                queue[length].y = cur.y;
                queue[length].d = cur.d + 1;
                visited[cur.x-1][cur.y] = 1;
                length++;
            }   
        }
        
        front++;
    }
}

int dists[MAZE_SIZE_X][MAZE_SIZE_Y]={};
char walls[MAZE_SIZE_X+1][MAZE_SIZE_Y+1] = {}; 
void legitProgram()
{   
    
    pc.printf("start!\r\n");
    /*
    
        <---- X SIZE --->
        ^
        |
        |
        Y
        |
        V
        
        (0,0) HERE
    */
    
    
    // add bottom and top borders
    for(int i=0;i<MAZE_SIZE_X;i++)
    {
        walls[i][0] |= 1;
        walls[i][MAZE_SIZE_Y] |= 1;
    }
    for(int i=0;i<MAZE_SIZE_Y;i++)
    {
        walls[0][i] |= 2;
        walls[MAZE_SIZE_X][i] |= 2;
    }
    // LSB is DOWN wall, 2nd LSB is LEFT wall
    
    Cell cur;
    cur.x=0;
    cur.y=0;
    
    if(SOLVEMODE)
        cur.d=DIR_LE;
    else
        cur.d=DIR_DO;
    
    Cell center;
    center.x = CENTER_X;
    center.y = CENTER_Y;
    center.d = 0;
    
    
    robot.rotateRight(100);
    turnDirRight(cur.d);
    robot.rotateRight(100);
    turnDirRight(cur.d);
    
    if(SOLVEMODE)
        walls[0][1] |= 1;
    else
        walls[1][0] |= 2;
    
    calcDists(dists,walls,center);
    
    while( (cur.x!=center.x) || (cur.y!=center.y))
    {
         /*for(int i = MAZE_SIZE_Y-1; i >= 0;i--)
        {
            for(int j=0;j<MAZE_SIZE_X;j++)
            {
                pc.printf("%d ",dists[j][i]);
            }   
            pc.printf("\n\r");
        }
        
        
        */
        
        // DETECT CURRENT WALLS
        if(!robot.gapLeft())
        {
            switch(cur.d){
                case DIR_UP:
                    walls[cur.x][cur.y] |= 2;
                break;  
                case DIR_RI:
                    walls[cur.x][cur.y+1] |= 1;
                break;
                case DIR_LE:
                    walls[cur.x][cur.y] |= 1;
                break;
                case DIR_DO:
                    walls[cur.x+1][cur.y] |= 2;
                break;
            }   
        }
        if(!robot.gapRight())
        {
            switch(cur.d){
                case DIR_UP:
                    walls[cur.x+1][cur.y] |= 2;
                break;  
                case DIR_RI:
                    walls[cur.x][cur.y] |= 1;
                break;
                case DIR_LE:
                    walls[cur.x][cur.y+1] |= 1;
                break;
                case DIR_DO:
                    walls[cur.x][cur.y] |= 2;
                break;
            }   
        }
        if(robot.wallFront())
        {
            
            switch(cur.d){
                case DIR_UP:
                    walls[cur.x][cur.y+1] |= 1;
                break;  
                case DIR_RI:
                    walls[cur.x+1][cur.y] |= 2;
                break;
                case DIR_LE:
                    walls[cur.x][cur.y] |= 2;
                break;
                case DIR_DO:
                    walls[cur.x][cur.y] |= 1;
                break;
            }   
        }    
        calcDists(dists,walls,center);
        
        // MOVE TO OPEN CELL THAT MOVES YOU CLOSER TO TARGET
        int minDis = 123123;
        int minx = cur.x;
        int miny = cur.y;
        int go = 0;
        // CHECK CELL ABOVE
        if((cur.y+1)<MAZE_SIZE_Y){
            if(  (walls[cur.x][cur.y+1]&1) == 0 && dists[cur.x][cur.y+1]<minDis)
            {
                minDis = dists[cur.x][cur.y+1];
                minx = cur.x;
                miny = cur.y+1;
                go = DIR_UP;
            }   
        }
        // CHECK CELL BELOW
        if((cur.y)>0){
            if(  (walls[cur.x][cur.y]&1) == 0 && dists[cur.x][cur.y-1]<minDis)
            {
                minDis = dists[cur.x][cur.y-1];
                minx = cur.x;
                miny = cur.y-1;
                go = DIR_DO;
            }   
        }
        // CHECK CELL TO RIGHT
        if((cur.x+1)<MAZE_SIZE_X){
            if(  (walls[cur.x+1][cur.y]&2) == 0 && dists[cur.x+1][cur.y]<minDis)
            {         
                minDis = dists[cur.x+1][cur.y];
                minx = cur.x+1;
                miny = cur.y;
                go = DIR_RI;
            }   
        }
        // CHECK CELL TO LEFT
        if((cur.x)>0){
            if(  (walls[cur.x][cur.y]&2) == 0 && dists[cur.x-1][cur.y]<minDis )
            {
                minDis = dists[cur.x-1][cur.y];
                minx = cur.x-1;
                miny = cur.y;
                go = DIR_LE;
            }   
        }
        
        //pc.printf("%d\r\n",go);
        // GO TO MINX MINY;
        int det = go - cur.d;
        switch(det)
        {
            case 1:
            case -3:
            case 2:
            case -2:
            while(cur.d!=go)
            {
                robot.rotateRight(100);
                turnDirRight(cur.d);  
                robot.attCal(); 
            }
            break;
            default:
            
            while(cur.d!=go)
            {
                robot.rotateLeft(100);
                turnDirLeft(cur.d);  
                robot.attCal(); 
            }
            break;
        }
        
        robot.moveFwd(600);
        
        switch(go)
        {
            case DIR_UP:
                cur.y++;
            break;   
            case DIR_DO:
                cur.y--;
            break;   
            case DIR_RI:
                cur.x++;
            break;   
            case DIR_LE:
                cur.x--;
            break;   
        }
        
    }
    
    
    
    
    
    /*
    // PREFILL DISTANCE ARRAY 
    for(int i = MAZE_SIZE_Y-1; i >= 0;i--)
    {
        for(int j=0;j<MAZE_SIZE_X;j++)
        {
            pc.printf("%d ",dists[j][i]);
        }   
        pc.printf("\n\r");
    }
    
        
/*
        robot.rotateRight(100);
        wait_ms(25);
        robot.rotateRight(100);
        wait_ms(25);
        robot.moveFwd(100);
        wait_ms(25);
        robot.moveFwd(100);
        wait_ms(25);
        robot.rotateLeft(100);
        wait_ms(25);
        robot.moveFwd(100);
        wait_ms(25);
        robot.rotateRight(100);
        wait_ms(25);
        robot.moveFwd(100);
        wait_ms(25);
        robot.moveFwd(100);
        wait_ms(25);
        */
        
}

int main() {

    robot.setIRs(&irll, &irfl, &irfr, &irrr);
    robot.setIREquil();
    robot.setEncP(&encr, &encl);

    Timer t;
    t.start();
    DigitalIn mybutton(USER_BUTTON);
    DigitalOut myled(LED1);
     
    while(t.read_ms()<2000)
    {  
        if (mybutton == 0) { // Button is pressed
            SOLVEMODE = 1; // Toggle the LED state  
        }

    }

    while(1){
        //wait(1);
        //pc.printf("%f %f %f %f\n\r",irll.read(),irfl.read(),irfr.read(),irrr.read());
         
        while(1)
        {  
            if (mybutton == 0) { // Button is pressed
                break; // Toggle the LED state  
            }
    
        }
        wait_ms(1000);
        //robot.attCal();
        //wait(2);
        legitProgram();
        
        
        
        
        //pc.printf("XD");
        //myLED=!myLED;
        
        //wait_ms(1000);
            
    }
}
