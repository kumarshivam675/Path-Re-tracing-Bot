// Include header files
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <libplayerc/playerc.h>
//#include <playerc.h>
// Random numbers generated for random walk movement by robots
int move_straight(int x,playerc_client_t *client,playerc_position2d_t *position2d)
{
int i;
for (i = 0; i<10; i++)
{
        // Wait for new data from server
        playerc_client_read(client);
        fprintf(stdout, "X: %3.2f, Y: %3.2f, Yaw: %3.2f \n",position2d->px, position2d->py, position2d->pa);
        // Random walk is continued till finding first marker
        if (0 != playerc_position2d_set_cmd_vel(position2d, x,0 ,0,0))
	        return -1;
        usleep(1000);
}
	while(0 != playerc_position2d_set_cmd_vel(position2d, 0,0 , 0,1))
		break;
	return 0;
}
int turn_left(playerc_client_t *client,playerc_position2d_t *position2d)
{
int i;
for (i = 0; i<9; i++)
{
	// Wait for new data from server
	playerc_client_read(client);
	fprintf(stdout, "X: %3.2f, Y: %3.2f, Yaw: %3.2f \n",position2d->px, position2d->py, position2d->pa);
	// Random walk is continued till finding first marker
	if (0 != playerc_position2d_set_cmd_vel(position2d, 0,0 , 1,0))
	return -1;
	usleep(1000);
}
	playerc_client_read(client);
	if (0 != playerc_position2d_set_cmd_vel(position2d, 0,0 , 0.8,0)) 
	return -1;
	while(0 != playerc_position2d_set_cmd_vel(position2d, 0,0 , 0,1))
		break;
}
int turn_right(playerc_client_t *client,playerc_position2d_t *position2d)
{
int i;
for (i = 0; i<9; i++)
{
	// Wait for new data from server
	playerc_client_read(client);
	fprintf(stdout, "X: %3.2f, Y: %3.2f, Yaw: %3.2f \n",position2d->px, position2d->py, position2d->pa);
	// Random walk is continued till finding first marker
	if (0 != playerc_position2d_set_cmd_vel(position2d, 0,0 , -1*1,0))
	return -1;
	usleep(1000);
}
	playerc_client_read(client);
	if (0 != playerc_position2d_set_cmd_vel(position2d, 0,0 ,-1*0.8,0)) 
	return -1;
	while(0 != playerc_position2d_set_cmd_vel(position2d, 0,0 , 0,1))
		break;
}
int
main(int argc, const char **argv)
{
int history[500];
int i,count=0;
playerc_client_t *client;
playerc_position2d_t *position2d;
int cycle, index=0;
double dist,angle,fidAngle = 0,lineAngle=0, fidDist=0, prevYaw=0,posAngle=0;
// Create a client and connect it to the server.
client = playerc_client_create(NULL, "localhost", 6665);
if (0 != playerc_client_connect(client))
return -1;
// Create and subscribe to a position2d device.
position2d = playerc_position2d_create(client, 0);
if (playerc_position2d_subscribe(position2d, PLAYER_OPEN_MODE))
return -1;
int choice,ch;
do
{
count+=1;
printf("\n1.Straight\t2.Left\t3.Right\t4.Reverse\t0.exit\n");
scanf("%d",&choice);
history[count-1]=choice;
switch(choice)
{
case 1 : {move_straight(1,client,position2d);break;}
case 2 : {turn_left(client,position2d);break;}
case 3 : {turn_right(client,position2d);break;}
case 4 : {move_straight(-1,client,position2d);break;}
case 0 : break;
default : {printf("Wrong Choice!!!!!\n"); break;}
}
//printf("\nFurther <0-yes> : ");
//scanf("%d",&ch);
}while(choice != 0 );
/*do
{
	move_straight(client,position2d);
	printf("\nDo You Want To Go Straight <0-yes> ? :");
	scanf("%d",&choice);
}while(choice == 0);*/
printf("\n\tRETRACING ITS PATH \n");
for(i=count-2;i>=0;i--)
{
switch(history[i])
{
case 4 : {move_straight(1,client,position2d);break;}
case 3 : {turn_left(client,position2d);break;}
case 2 : {turn_right(client,position2d);break;}
case 1 : {move_straight(-1,client,position2d);break;}
}
}
while(1)
{
	if (0 != playerc_position2d_set_cmd_vel(position2d, 0,0 , 0,1))
	return -1;
}
playerc_position2d_unsubscribe(position2d);
playerc_position2d_destroy(position2d);
playerc_client_disconnect(client);
playerc_client_destroy(client);
return 0;
}

