/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>

#define IMG_SIZE NUM_COLORS*(OSC_CAM_MAX_IMAGE_WIDTH/2)*(OSC_CAM_MAX_IMAGE_HEIGHT/2)
#define slowImplementation
#define Border 6
#define Paramk 5

const int nc = OSC_CAM_MAX_IMAGE_WIDTH / 2;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT / 2;

int TextColor;
int avgDxy[3][IMG_SIZE];
int helpBuf[IMG_SIZE];

// Declare Funcions
void CalcDeriv();
void AvgDeriv(int Index);
void EdgeStrength();
void locMax();

void ResetProcess() {
	//called when "reset" button is pressed
	if (TextColor == CYAN)
		TextColor = MAGENTA;
	else
		TextColor = CYAN;
}

void ProcessFrame() {
	uint32 t1, t2;
	//char Text[] = "hallo world";
	//initialize counters
	if (data.ipc.state.nStepCounter == 1) {
		//use for initialization; only done in first step
		memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);
		memset(avgDxy, 0, sizeof(avgDxy)); // set avgDxy values to zero

		TextColor = CYAN;
	} else {
		//example for time measurement
		t1 = OscSupCycGet();
		//example for copying sensor image to background image
		//memcpy(data.u8TempImage[BACKGROUND], data.u8TempImage[SENSORIMG], IMG_SIZE);
		//example for time measurement

		OscLog(INFO, "Call CalcDeriv() \n");
		CalcDeriv();
		OscLog(INFO, "EdgeStrength() \n");
		EdgeStrength();
		OscLog(INFO, "locMax() \n");
		locMax();


		t2 = OscSupCycGet();

		//example for log output to console
		OscLog(INFO, "required = %d us\n", OscSupCycToMicroSecs(t2 - t1));

		//example for drawing output
		//draw line
		/*
		 DrawLine(10, 100, 200, 20, RED);
		 //draw open rectangle
		 DrawBoundingBox(20, 10, 50, 40, false, GREEN);
		 //draw filled rectangle
		 DrawBoundingBox(80, 100, 110, 120, true, BLUE);
		 DrawString(200, 200, strlen(Text), TINY, TextColor, Text);
		 */
	}
}

void CalcDeriv() {
	int c, r;
	for (r = nc; r < nr * nc - nc; r += nc) {/* we skip the first and last line */
		for (c = 1; c < nc - 1; c++) {
			/* do pointer arithmetics with respect to center pixel location */
			unsigned char* p = &data.u8TempImage[SENSORIMG][r + c];
			/* implement Sobel filter in x-direction */
			int dx = -(int) *(p - nc - 1) + (int) *(p - nc + 1)	// nc entspricht Anz Pixel einer Zeile
			- 2 * (int) *(p - 1) + 2 * (int) *(p + 1) - (int) *(p + nc - 1)
					+ (int) *(p + nc + 1);

			/* implement Sobel filter in y-direction */
			int dy = -(int) *(p - nc - 1) - 2 * (int) *(p - nc)
					- (int) *(p - nc + 1) + (int) *(p + nc - 1)
					+ 2 * (int) *(p + nc) + (int) *(p + nc + 1);

			avgDxy[0][r + c] = dx * dx;
			avgDxy[1][r + c] = dy * dy;
			avgDxy[2][r + c] = dx * dy;

			//data.u8TempImage[BACKGROUND][r+c] = MAX(0, MIN(255,(dx*dx>>10)));
			//data.u8TempImage[THRESHOLD][r+c] = MAX(0, MIN(255,(avgDxy[1][r+c]>>10)));

			//AvgDeriv(0); // param 0 for: <dx*dx>
			//AvgDeriv(1); // param 1 for: <dy*dy>

			//data.u8TempImage[BACKGROUND][r+c] = MAX(0, MIN(255,(avgDxy[0][r+c]>>10)));
			//data.u8TempImage[THRESHOLD][r+c] = MAX(0, MIN(255,(avgDxy[1][r+c]>>10)));

			// Calc EdgeStrength
			//EdgeStrength();

		}
	}
}

#ifdef slowImplementation

void AvgDeriv(int Index) {
//do average in x-direction
	int c, r;
	for (r = nc; r < nr * nc - nc; r += nc) {/* we skip first and last lines (empty) */
		for (c = Border + 1; c < nc - (Border + 1); c++) {/* +1 because we have one empty border column */
			/* do pointer arithmetics with respect to center pixel location */
			int* p = &avgDxy[Index][r + c];
			int sx = (*(p-6) + *(p+6)) + ((*(p-5) + *(p+5)) <<2) + (*(p-4) +*(p+4))*11 +
   					 (*(p-3) + *(p+3))*27 + (*(p-2)+ *(p+2))*50 + (*(p-1) + *(p+1))*72 + (*p)*82;

			//now averaged
			helpBuf[r + c] = (sx >> 8); // Auflösung 1/256
		}
	}

//do average in y-direction
	for (r = nc*(Border+1); r < nr * nc - nc*(Border+1); r += nc) {/* we skip first and last lines (empty) */
		for (c = Border + 1; c < nc - (Border + 1); c++) {/* +1 because we have one empty border column */
			/* do pointer arithmetics with respect to center pixel location */
			int* p = &helpBuf[r +c];
			int sy = (*(p-6*nc) + *(p+6*nc)) + ((*(p-5*nc) + *(p+5*nc)) << 2) + (*(p-4*nc) + *(p+4*nc))*11 +
					 (*(p-3*nc) + *(p+3*nc))*27 + (*(p-2*nc) + *(p+2*nc))*50 + (*(p-1*nc) + *(p+1*nc))*72 + (*p)*82;

			//now averaged
			avgDxy[Index][r + c] = (sy >> 8);
			//data.u8TempImage[BACKGROUND][r+c] = MAX(0,MIN(255,(avgDxy[Index][r+c] >> 11)));
		}
	}
}


#endif


void EdgeStrength()
{
	int r,c;

	AvgDeriv(0); // param 0 for: <dx*dx>
	AvgDeriv(1); // param 1 for: <dy*dy>
	AvgDeriv(2); // param 2 for: <dx * dy>


	for(r=nc*(Border+1); r < nr*nc-nc*(Border+1);r+=nc){
		for(c=Border+1; c< nc-(Border+1);c++)
		{
		  int dIx2 = (avgDxy[0][r+c] >>7);
		  int dIy2 = (avgDxy[1][r+c] >>7);
		  int dIxy = (avgDxy[2][r+c] >>7);

		  avgDxy[0][r+c] = (dIx2*dIy2 - (dIxy*dIxy)) - ((Paramk *((dIx2+dIy2)*(dIx2+dIy2)))>>7);

		  data.u8TempImage[THRESHOLD][r+c] = MAX(0,MIN(255,(avgDxy[0][r+c] >> 10)));
		  //data.u8TempImage[THRESHOLD][r+c] = MAX(0, MIN(255,(avgDxy[1][r+c]>>10)));
		}
	}
}

void locMax()
{
	int c, r;
	int tempMax = 0;

	for (r = nc; r < nr * nc - nc; r += nc) {/* we skip first and last lines (empty) */
		for (c = Border + 1; c < nc - (Border + 1); c++) {/* +1 because we have one empty border column */
			/* do pointer arithmetics with respect to center pixel location */
			int* p = &avgDxy[0][r + c];
			tempMax = 0;
			for(int i =6; i>=0 ; i--)
			{
				if(*(p-i) > tempMax)
				{
					tempMax = *(p-i);
				}
				if(*(p+i) > tempMax)
				{
					tempMax = *(p+i);
				}
				helpBuf[r + c] = tempMax;
				//*(p-i) = tempMax;
				//*(p+i) = tempMax;
			}
		}
	}

	for (r = nc*(Border+1); r < nr * nc - nc*(Border+1); r += nc) {/* we skip first and last lines (empty) */
		for (c = Border + 1; c < nc - (Border + 1); c++) {/* +1 because we have one empty border column */
			/* do pointer arithmetics with respect to center pixel location */
			//int* p = &avgDxy[0][r + c];
			int* p = &helpBuf[r +c];
			tempMax = 0;
			for(int i =6; i>=0 ; i--)
			{
				if(*(p-i*nc) > tempMax)
				{
					tempMax = *(p-i*nc);
				}
				if(*(p+i*nc) > tempMax)
				{
					tempMax = *(p+i*nc);
				}
				avgDxy[0][r + c] = tempMax;
				//*(p-i*nc) = tempMax;
				//*(p+i*nc) = tempMax;
			}

			//now averaged
			data.u8TempImage[BACKGROUND][r+c] = MAX(0,MIN(255,(avgDxy[0][r+c] >> 10)));
		}
	}
	  //data.u8TempImage[BACKGROUND][r+c] = MAX(0,MIN(255,(avgDxy[0][r+c] >> 11)));
}
