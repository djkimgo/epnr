// ENM.cpp : Defines the entry point for the console application.

#include <cv.h>
#include <highgui.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include "CFindButtons.h"

int main(int argc, char **argv)
{
	char user_str[256];
	char file_prefix[256];
	char file_ext[256];

	char debug_path[] = {"C:\\myjava\\test6\\myout"};
	double t;

	int i,j,k;
	int** position = 0;
	int button_id[2]={0,2};

	IplImage *o_img;

	CvFont font;
	cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX, 0.25f, 0.25f, 0, 1 );

	CFindButtons myButton[3];

	for (j=button_id[0]; j<button_id[1]; j++) {
		myButton[j].set_type_buttons(j);	// 0: 1-8, 1: B1-B3, 2: Up & Down
		myButton[j].set_zoom_factor( 1.5f, 1.5f );	// Zoom Factor for x- and y- directional component
		myButton[j].set_basepath( "C:\\myjava\\test6" );
		myButton[j].debug_on();	// Save Debug Image
		//myButton[j].show_debug();
		myButton[j].load_templates();	// Load Templates
	}
	
	sprintf( file_prefix, "b1_" );
	sprintf( file_ext, "png" );

	i = 50;

	IplImage* r_img1;
	IplImage* r_img;

	while (1) {
		printf(">>%s%d.%s\n", file_prefix, i+1, file_ext);

		t = cvGetTickCount();

		sprintf( user_str, "%s%s%d.%s", myButton[0].filepath, file_prefix, i+1, file_ext );
		r_img1 = cvLoadImage( user_str, 0 );
		if (!r_img1)	break;

		r_img = cvCloneImage( r_img1 );

		o_img = cvCreateImage( cvGetSize(r_img), 8, 3 );
		cvSetZero( o_img );
		cvMerge( r_img, r_img, r_img, 0, o_img );
		
		for (k=button_id[0]; k<button_id[1]; k++) {

			sprintf( user_str, "%s%d.%s", file_prefix, i+1, file_ext );
			myButton[k].set_debug_info( user_str );

			/* call detect buttons */		
			//myButton.learn_buttons( r_img );

			position = myButton[k].detect_buttons( r_img );

			if (position == 0)
				continue;

			//for (int j=0; j<myButton.number_of_buttons; j++) {
			for (j=0; position[j][0] != -99;j++) {
				if ((position[j][0] != -1) && (position[j][1] != -1)) {

					float a_ratio = ((float)(position[j][2]-position[j][0]))/((float)(position[j][3]-position[j][1]));

					if (a_ratio < 0.9f)
						continue;

					cvDrawRect( o_img, cvPoint(position[j][0],position[j][1]), cvPoint(position[j][2],position[j][3]), CV_RGB(255*((k==0) ? 1 : 0),255*((k==1) ? 1 : 0),255*((k==2) ? 1 : 0)), 1 );
					sprintf( user_str, "%s%d", ((k==0) ? "" : ((k==1) ? "B" : ((position[j][4]==0) ? "U" : "D"))), position[j][4]+1 );
					cvPutText( o_img, user_str, cvPoint(((k==0) ? position[j][0] : ((k==1) ? (position[j][0]+position[j][2])/2 : position[j][2])),position[j][3]), &font, CV_RGB(0,255,255) );
				}
			}
		}

		sprintf( user_str, "%s\\%s%d.%s", debug_path, file_prefix, i+1, file_ext );
		cvSaveImage( user_str, o_img );

		t = cvGetTickCount() - t;		
		printf( "frame#%d is processed in %g ms.\n", i+1, t/((double)cvGetTickFrequency()*1000.) );

		if (position)
			delete [] position;
		cvReleaseImage( &o_img );
		cvReleaseImage( &r_img );
		cvReleaseImage( &r_img1 );
		i++;		
	};

	//system("pause");

	return 0;
}