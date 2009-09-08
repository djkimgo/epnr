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

	char debug_path[] = {"D:\\김박사연구폴더\\[KWU.Face_Robot]\\Elevator\\test5\\templates2"};
	double t;

	IplImage *o_img;

	CvFont font;
	cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX, 0.5f, 0.5f, 0, 2 );

	CFindButtons myButton;
	//myButton.resize_factor(1);
	myButton.set_basepath( "D:\\김박사연구폴더\\[KWU.Face_Robot]\\Elevator\\test5" );
	myButton.debug_on();
	myButton.show_debug();
	//myButton.load_templates();
	
	sprintf( file_prefix, "" );
	sprintf( file_ext, "jpg" );

	int i = 0;

	//while (1) 
	{
		printf(">>%s%d.%s\n", file_prefix, i+1, file_ext);
		
		sprintf( user_str, "%s%s%d.%s", myButton.filepatht, file_prefix, i+1, file_ext );
		IplImage* r_img1 = cvLoadImage( user_str, 0 );
		//if (!r_img1)	break;

		//IplImage* r_img = cvCreateImage( cvSize(r_img1->width/2, r_img1->height/2), 8, 1 );
		//cvResize( r_img1, r_img ); 

		IplImage* r_img = cvCloneImage( r_img1 );

		o_img = cvCreateImage( cvGetSize(r_img), 8, 3 );
		cvSetZero( o_img );
		cvMerge( r_img, r_img, r_img, 0, o_img );

		sprintf( user_str, "%s%d.%s", file_prefix, i+1, file_ext );
		myButton.set_debug_info( user_str );

		/* call detect buttons */
		t = cvGetTickCount();
		//int** position = myButton.detect_buttons( r_img );
		myButton.learn_buttons( r_img );

		/*for (int j=0; j<myButton.number_of_buttons; j++) {
			if ((position[j][0] != -1) && (position[j][1] != -1))
			cvDrawRect( o_img, cvPoint(position[j][0],position[j][1]), cvPoint(position[j][2],position[j][3]), CV_RGB(255,0,0), 2 );
			sprintf( user_str, "%d", j+1 );
			cvPutText( o_img, user_str, cvPoint(position[j][2]+5,position[j][3]), &font, CV_RGB(0,255,255) );
		}

		sprintf( user_str, "%s\\%s%d.%s", debug_path, file_prefix, i+1, file_ext );
		cvSaveImage( user_str, o_img );*/

		t = cvGetTickCount() - t;		
		printf( "frame#%d is processed in %g ms.\n", i+1, t/((double)cvGetTickFrequency()*1000.) );

		//delete [] position;
		cvReleaseImage( &o_img );
		cvReleaseImage( &r_img );
		cvReleaseImage( &r_img1 );
		i++;
	};

	return 0;
}