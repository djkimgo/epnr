// DJFaceAPI.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include "cv.h"
#include "highgui.h"
#include "cvmser.h"

//using namespace std;

#define	M_PI	(3.1415926535897932384626433832795)

int main(int argc, char* argv[])
{
	IplImage* img = 0;
	IplImage* o_img = 0;
	IplImage* t_img[10];
	cvNamedWindow("Display0");

	CvSeq* contours;
	CvMemStorage* storage;

	int i, j;
	char user_str[256];
	int img_no = 1;

	storage = cvCreateMemStorage(0);

	for (i=8; i<10; i++) {
		sprintf( user_str, "C:\\myjava\\faces\\t%d.jpg", i+1 );
		t_img[i] = cvLoadImage( user_str, 0 );
	}

	int sW, sH, sw, sh;
	double tt;

	while (1) {

		sprintf( user_str, "c:\\myjava\\faces\\img%03d.jpg", img_no );
		img = cvLoadImage( user_str, 0 );
		if (!img) break;

		o_img = cvCreateImage(cvSize(img->width*2,img->height),8,3);
		cvSetImageROI( o_img, cvRect(0,0,img->width,img->height) );
		cvMerge( img, img, img, 0, o_img );
		cvResetImageROI( o_img );

		uchar* rsptr = (uchar*)o_img->imageData;

		IplImage* wh_img = cvCreateImage( cvSize(img->width,img->height), 8, 1 );
		cvSetZero( wh_img );
		cvResize( img, wh_img );

		sW = wh_img->width; sH = wh_img->height;

		IplImage* tt_img;
		IplImage* mt_img;

		tt = (double)cvGetTickCount();

		for (i=8; i<10; i++) {

			tt_img = cvCloneImage( t_img[i] );
			
			sw = tt_img->width; sh = tt_img->height;			
			
			mt_img = cvCreateImage( cvSize(sW-sw+1,sH-sh+1), 32, 1 );
			cvSetZero( mt_img );

			/*
				Match Template test
			*/
			cvMatchTemplate( wh_img, tt_img, mt_img, CV_TM_SQDIFF_NORMED);

			double min_val = 2.0f; 
			double max_val = -0.1f;
			CvPoint min_loc, max_loc;

			//cvMinMaxLoc( mt_img, &min_val, &max_val, &min_loc, &max_loc );

			double sum_val = 0.0f;

			for (j=0; j<10; j++) {
		
				min_val = 2.0f; max_val = -0.1f;
				cvMinMaxLoc( mt_img, &min_val, &max_val, &min_loc, &max_loc );
				
				if (min_val == 1.0f)
					continue;

				sum_val += min_val;

				//float out_elem[4] = {num_id, min_val, min_loc.x, min_loc.y};
				//cvSeqPush( output, out_elem);

				cvSetReal2D( mt_img, min_loc.y, min_loc.x, max_val );

				//printf("[%d] %f @ %d,%d - %f\n",i, min_val, min_loc.x, min_loc.y, max_val );
				//mt_img->imageData[min_loc.x + min_loc.y * mt_img->width] = max_val;
				cvCircle( o_img, cvPoint(min_loc.x+sw/2,min_loc.y+sh/2), 2, CV_RGB(255-(int)((float)i/8.0f*255.0f),0,(int)((float)i/8.0f*255.0f)), -1 );
			}

			//cvCircle( o_img, cvPoint(min_loc.x + sw/2,min_loc.y + sh/2), 3, CV_RGB(0,255,0), -1 );
			
			printf( "[temp#%d] %.3f \n", i, sum_val/10.0f);
		}

		tt = (double)cvGetTickCount() - tt;		
		printf( "frame#%d is processed in %g ms.\n", img_no+1, tt/((double)cvGetTickFrequency()*1000.) );


		/*
			MSER test
		*/
		/*cvExtractMSER( img, NULL, &contours, storage, 
			cvMSERParams( 1, cvRound(.0001*img->width*img->height), 
			cvRound(.04*img->width*img->height), .25, .2) );		

		for (  i = contours->total-1; i >= 0; i-- ) {
			CvSeq* r = *(CvSeq**)cvGetSeqElem( contours, i );

			for (  j = 0; j < r->total; j++ ) {
				CvPoint* pt = CV_GET_SEQ_ELEM( CvPoint, r, j );
				rsptr[(pt->x + img->width)*3 + pt->y*o_img->widthStep] = 255-(uchar)((float)j/(float)r->total*255.0f);
				rsptr[(pt->x + img->width)*3+1 + pt->y*o_img->widthStep] = (uchar)((float)j/(float)r->total*255.0f);
				rsptr[(pt->x + img->width)*3+2 + pt->y*o_img->widthStep] = (uchar)((float)j/(float)r->total*255.0f);
			}
		}
		*/

		cvShowImage("Display0", o_img);

		cvWaitKey();

		cvReleaseImage( &o_img );

		img_no++;
	}

	cvDestroyAllWindows();

	cvReleaseImage( &img );

	return 0;
}

	