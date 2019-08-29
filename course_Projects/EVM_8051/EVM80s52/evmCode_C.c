// Program to make voting machine using seven segment
#include <reg51.h>
#define msec 1

sbit switch_1=P3^2;//Input pins for four candidates
sbit switch_2=P3^3;
sbit switch_3=P3^4;
sbit switch_4=P3^5;

sbit dig_ctrl_4=P1^0;//Declare the control pins of seven segments
sbit dig_ctrl_3=P1^1;
sbit dig_ctrl_2=P1^2;
sbit dig_ctrl_1=P1^3;

unsigned int vote_1,vote_2,vote_3,vote_4;

unsigned int digi_val[10]={0x40,0xF9,0x24,0x30,0x19,0x12,0x02,0xF8,0x00,0x10};
unsigned int dig_1,dig_2,dig_3,dig_4;



void delay(unsigned int count)// Time delay function
{
	unsigned int j,k;
	for (j=0;j<=count;j++)
	for (k=0;k<=50;k++);
}

void digi_out(unsigned int current_num)// Funtion to display total votes
{
	unsigned int dig_disp;
	dig_disp=current_num;
	P2 = digi_val[current_num];
	delay(msec);
}


void calc_vote()// Funtion to count the number of votes
{
	while(1)
	{
		if (switch_1==0)
		{
			while (switch_1 == 0);//check if switch 1 is pressed
			{
				vote_1 = vote_1 + 1;
				if(vote_1==10)
				vote_1=0;
			}
		}

		if (switch_2==0)//check if switch 2 is pressed
		{
			while (switch_2 == 0);
			{
				vote_2 = vote_2 + 1;
				if(vote_2==10)
				vote_2=0;
			}
		}

		if (switch_3==0)//check if switch 3 is pressed
		{
			while (switch_3 == 0);
			{
				vote_3 = vote_3 + 1;
				if(vote_3==10)
				vote_3=0;
			}
		}

		if (switch_4==0)//check if switch 4 is pressed
		{
			while (switch_4 == 0);
			{
				vote_4 = vote_4 + 1;
				if(vote_4==10)
				vote_4=0;
			}
		}

	dig_ctrl_1 = 1;
	dig_ctrl_3 = dig_ctrl_2 = dig_ctrl_4 = 0;
	digi_out(vote_1);
	dig_ctrl_2 = 1;
	dig_ctrl_4 = dig_ctrl_3 = dig_ctrl_1 = 0;
	digi_out(vote_2);
	dig_ctrl_3 = 1;
	dig_ctrl_2 = dig_ctrl_4 = dig_ctrl_1 = 0;
	digi_out(vote_3);
	dig_ctrl_4 = 1;
	dig_ctrl_3 = dig_ctrl_2 = dig_ctrl_1 = 0;
	digi_out(vote_4);
	}
}


void main()
{
vote_1 = vote_2 = vote_3 = vote_4 = 0;
switch_1 = switch_2 = switch_3 = switch_4 = 1;// Initialize the input pins
while(1)
{
calc_vote();
}
}