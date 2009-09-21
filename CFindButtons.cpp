#include <cv.h>
#include <highgui.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include "cvmser.h"
#include "CFindButtons.h"

using namespace std;

#define	M_PI	(3.1415926535897932384626433832795)

CFindButtons::CFindButtons() 
{
	// param
	hu1_range[0] = 0.002f; hu1_range[1] = 0.01f;
	hu2_range[0] = 0.0f; hu2_range[1] = 0.0001f;
	cdist_r = 0.2f;
	aratio_r[0] = 0.1f; aratio_r[1] = 0.5f;
	cross_p = 5;
	th_th = 60.0f;
	ecc_range[0] = 0.46f;
	ecc_range[1] = 1.0f;

	/* constraint list : 
	1. overlap with SURF points
	2. Hu moments hu1[0]< hu1 < hu1[1] hu2[0] < hu2 < hu2[1]
	3. |box_center - centre_of_mass| > r * 0.2
	4. area_ratio (area_box/area_circle) < 0.3
	5. cross_FOV w/ 5 pixels
	6. angles ~ 60.0
	7. eccentricity - 0.76~0.98
	*/
	constraint[0] = false;	// overlap
	constraint[1] = true;	// hu moments
	constraint[2] = false;	// box_center_dist
	constraint[3] = true;	// area_ratio
	constraint[4] = true;	// cross FOV
	constraint[5] = false;	// angle
	constraint[6] = true;	// eccentricity
	constraint[7] = false;
	constraint[8] = false;
	constraint[9] = false;

	/*for (int i=0; i<10; i++) {
		constraint[i] = false;
	}*/

	storage = cvCreateMemStorage(0);

	number_of_buttons = 8;	// type_buttons = 0
	type_buttons = 0;
	cobject = new CObject[number_of_buttons];

	residual_MSER = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvRect), storage );

	debug_image = true;
	show_dbg = false;
	resize_num = 2;

	t_img = 0; st_img = 0; t_img_tiny = 0;

	zoom_x = 1.0f; zoom_y = 1.0f;
}

void CFindButtons::load_templates()
{
	int buttons_start, buttons_end;

	switch ( type_buttons ) {
		case 0:	// 1-8
			// param
			hu1_range[0] = 0.5f; hu1_range[1] = 3.5f;
			hu2_range[0] = 0.005f; hu2_range[1] = 10.0f;
			ecc_range[0] = 0.46f;
			ecc_range[1] = 0.99f;

			number_of_buttons = 8;
			buttons_start = 0;
			buttons_end = 8;
			break;
		case 1:	// B1-B3
			hu1_range[0] = 0.29f; hu1_range[1] = 2.3f;
			hu2_range[0] = 0.04f; hu2_range[1] = 1.35f;
			ecc_range[0] = 0.65f;
			ecc_range[1] = 1.0f;

			number_of_buttons = 3;
			buttons_start = 8;
			buttons_end = 11;
			break;
		case 2:	// UP & DOWN
			hu1_range[0] = 0.5f; hu1_range[1] = 2.4f;
			hu2_range[0] = 0.1f; hu2_range[1] = 5.0f;
			ecc_range[0] = 0.93f;
			ecc_range[1] = 0.991f;

			number_of_buttons = 2;
			buttons_start = 11;
			buttons_end = 13;
			break;
	}

	/* template loading */
	t_img = new IplImage*[number_of_buttons];
	st_img = new IplImage*[number_of_buttons];
	for (int i=buttons_start; i<buttons_end; i++) {
		sprintf( user_str, "%s%d.jpg", filepatht, i+1 );		
		t_img[i-buttons_start] = cvLoadImage( user_str, 0 );
		sprintf( user_str, "%sstest%d.jpg", filepatht, i+1 );		
		st_img[i-buttons_start] = cvLoadImage( user_str, 0 );
	}
	t_img_tiny = cvCreateImage( cvSize(t_img[0]->width/number_of_buttons, t_img[0]->height/number_of_buttons), 8, 1 );
}

CFindButtons::~CFindButtons() 
{	
	if (t_img_tiny) {
		cvReleaseImage( &t_img_tiny );
	}

	if (st_img) {
		for (int i=0; i<number_of_buttons; i++) {
			cvReleaseImage( &(st_img[i]) );
			cvReleaseImage( &(t_img[i]) );
		}

		delete [] t_img;
		delete [] st_img;
	}
	
	delete [] cobject;
}

void CFindButtons::set_basepath( char *str )
{ 
	sprintf( basepath, "%s", str);

	sprintf( filepath, "%s\\gray\\", basepath);
	sprintf( filepath2, "%s\\mser\\", basepath);
	sprintf( filepath3, "%s\\surf\\", basepath);
	sprintf( filepathx, "%s\\sharp\\", basepath);
	sprintf( filepatht, "%s\\templates2\\", basepath);
	sprintf( filepatho, "%s\\mser_out\\", basepath);
}

void CFindButtons::set_debug_info( char *str )
{
	sprintf( debugimg, "%s", str );
}

float CFindButtons::overlap_degree( CObject obj1, CObject obj2 ) 
{
	float ovd;

	ovd = max(0,min(obj1.lr.x,obj2.lr.x)-max(obj1.ul.x,obj2.ul.x))
		* max(0,min(obj1.lr.y,obj2.lr.y)-max(obj1.ul.y,obj2.ul.y)) 
		/ (((obj1.lr.y-obj1.ul.y)*(obj1.lr.x-obj1.ul.x)+((obj2.lr.y-obj2.ul.y)*(obj2.lr.x-obj2.ul.x)))/2.0f);

	return (exp(-(1.0f - ovd )));
}

float CFindButtons::overlap_degree( CvRect obj1, CvRect obj2 ) 
{
	float ovd;

	ovd = max(0,min(obj1.x+obj1.width,obj2.x+obj2.width)-max(obj1.x,obj2.x))
		* max(0,min(obj1.y+obj1.height,obj2.y+obj2.height)-max(obj1.y,obj2.y)) 
		/ (((obj1.height)*(obj1.width)+((obj2.height)*(obj2.width)))/2.0f);

	return (exp(-(1.0f - ovd )));
}

void CFindButtons::match_number_image( const IplImage* t_img, const IplImage* s_img, const int width, const int height, double &min_v, CvPoint &min_r )
{
	IplImage *wh_img, *tt_img, *mt_img;
	int sW, sH, sw, sh;
	double min_val, max_val;
	CvPoint min_loc, max_loc;

	wh_img = cvCloneImage( t_img );
	tt_img = cvCreateImage( cvSize( width, height ), 8, 1 );
	cvSetZero( tt_img );
	cvResize( s_img, tt_img );

	sW = wh_img->width; sH = wh_img->height;
	sw = tt_img->width; sh = tt_img->height;
	
	mt_img = cvCreateImage( cvSize(sW-sw+1,sH-sh+1), 32, 1 );
	cvSetZero( mt_img );

	/* match template between resized template and current segmented image */
	cvMatchTemplate( wh_img, tt_img, mt_img, CV_TM_SQDIFF_NORMED);

	min_val = 2.0f; max_val = -0.1f;
	cvMinMaxLoc( mt_img, &min_val, &max_val, &min_loc, &max_loc );

	min_v = min_val;
	min_r = min_loc;

	cvReleaseImage( &mt_img );
	cvReleaseImage( &tt_img );
	cvReleaseImage( &wh_img );
}

int** CFindButtons::detect_buttons( IplImage *r_img )
{
	/* init return array */
	int **position = new int*[this->number_of_buttons];
	for (int i=0; i<this->number_of_buttons; i++) {
		position[i] = new int[4];
		for (int j=0; j<4; j++)
			position[i][j] = -1;
	}

	IplImage* g_img = cvCreateImage( cvGetSize(r_img), 8, 1 );
	IplImage* c_img = cvCreateImage( cvGetSize(r_img), 8, 3 );

	if (r_img->nChannels == 3) {
		cvCopyImage( r_img, c_img );
		cvConvertImage( r_img, g_img, CV_BGR2GRAY );
	} else {
		cvCopyImage( r_img, g_img );
		cvMerge( r_img, r_img, r_img, 0, c_img );
	}

	/* set MSER parameters */
	cvExtractMSER( g_img, NULL, &contours, storage, 
		cvMSERParams( 1 /* delta */, cvRound(.0001*r_img->width*r_img->height) /* min area */, 
		cvRound(.04*r_img->width*r_img->height) /* max area */, .25 /* max variation */, .2 /* min diversity */ ) );

	uchar* rsptr = (uchar*)c_img->imageData;

	IplImage *seg_img = cvCreateImage(cvGetSize(r_img),8,1);
	cvSetZero( seg_img );

	CvPoint box_xy = cvPoint(0,0);
	int count = 0;
	cvClearSeq( residual_MSER );

	for ( int i = contours->total-1; i >= 0; i-- ) {	/* check ith region/blob */
		CvSeq* r = *(CvSeq**)cvGetSeqElem( contours, i );

		//float pix_sum = 0.0f;
		//int min_p = 1e3, max_p = -1;
		//for ( int j = 0; j < r->total; j++ ) {
		//	CvPoint* pt = CV_GET_SEQ_ELEM( CvPoint, r, j );
		//	char gg = g_img->imageData[pt->x+pt->y*g_img->widthStep];
		//	/*if (gg<0)
		//		gg += 120;*/
		//	pix_sum += (float)((float)gg+128.0f)/255.0f;

		//	min_p = MIN(min_p,gg);
		//	max_p = MAX(max_p,gg);
		//}
		//pix_sum /= (float)(r->total);

		//if (pix_sum > 0.5f)
		//	continue;

		/* min enclosing rect & circle */
		box = cvMinAreaRect2( r, 0 );
		cvBoxPoints( box, box_vtx );
		cvMinEnclosingCircle( r, &center, &radius );			
		
		/* moments */
		IplImage *m_img = cvCloneImage( g_img );
		cvSetZero( m_img );
		for ( int j = 0; j < r->total; j++ ) {
			CvPoint* pt = CV_GET_SEQ_ELEM( CvPoint, r, j );
			//printf("%d \n",m_img->imageData[pt->x+pt->y*m_img->widthStep]);
			m_img->imageData[pt->x+pt->y*m_img->widthStep] = 127;
			//printf("%d \n",m_img->imageData[pt->x+pt->y*m_img->widthStep]);
		}

		cvMoments( m_img, &moments, 1 );			
		cvGetHuMoments( &moments, &hu_moments);

		cvReleaseImage( &m_img );

		CvPoint centre_of_mass = cvPoint( moments.m10/moments.m00, 
			moments.m01/moments.m00 );

		CvPoint box_center = cvPoint((cvRound(box_vtx[0].x)+cvRound(box_vtx[2].x))/2,
			(cvRound(box_vtx[0].y)+cvRound(box_vtx[2].y))/2);

		float mu20 = cvGetCentralMoment( &moments, 2, 0 );
		float mu02 = cvGetCentralMoment( &moments, 0, 2 );
		float mu00 = cvGetCentralMoment( &moments, 0, 0 );
		float mu11 = cvGetCentralMoment( &moments, 1, 1 );

		float theta = 0.5f*atan2(2.0f*mu11/mu00, mu20/mu00-mu02/mu00)*180.0f/M_PI;
		float lambda1 = (mu20/mu00+mu02/mu00)/2.0f + sqrt(4.0f*mu11/mu00*mu11/mu00+(mu20/mu00-mu02/mu00)*(mu20/mu00-mu02/mu00))/2.0f;
		float lambda2 = (mu20/mu00+mu02/mu00)/2.0f - sqrt(4.0f*mu11/mu00*mu11/mu00+(mu20/mu00-mu02/mu00)*(mu20/mu00-mu02/mu00))/2.0f;
		float eccent = sqrt(1-lambda2/lambda1);

		if (show_dbg)
			printf("[%s:%d] %f %f %.2f %f\n", debugimg, i, hu_moments.hu1, hu_moments.hu2, theta, eccent );

		/**/
		if ((((hu_moments.hu1 < hu1_range[0]) || (hu_moments.hu1 > hu1_range[1])) || ((hu_moments.hu2 < hu2_range[0]) || (hu_moments.hu2 > hu2_range[1]))) && constraint[1])
			continue;

		float cdist = sqrt((float)((box_center.x - centre_of_mass.x)*(box_center.x - centre_of_mass.x) 
			+ (box_center.y - centre_of_mass.y)*(box_center.y - centre_of_mass.y)));

		/**/
		if ((cdist > radius * cdist_r) && constraint[2])
			continue;

		/*float a_ratio = (sqrt((float)((cvRound(box_vtx[0].x)-cvRound(box_vtx[1].x))*(cvRound(box_vtx[0].x)-cvRound(box_vtx[1].x))
			+(cvRound(box_vtx[0].y)-cvRound(box_vtx[1].y))*(cvRound(box_vtx[0].y)-cvRound(box_vtx[1].y))))*sqrt((float)((cvRound(box_vtx[1].x)-cvRound(box_vtx[2].x))*(cvRound(box_vtx[1].x)-cvRound(box_vtx[2].x))
			+(cvRound(box_vtx[1].y)-cvRound(box_vtx[2].y))*(cvRound(box_vtx[1].y)-cvRound(box_vtx[2].y)))))/(M_PI*radius*radius);*/

		float a_ratio = mu00/(M_PI*radius*radius);

		/**/
		if (((a_ratio < aratio_r[0]) || (a_ratio > aratio_r[1])) && constraint[3])
		//if (((a_ratio < aratio_r[0])) && constraint[3])
			continue;

		CvPoint min_bpt = cvPoint(1e3,1e3), max_bpt = cvPoint(-1,-1);
		for ( int k=0; k<4; k++)
		{
			pt.x = cvRound(box_vtx[k].x);
			pt.y = cvRound(box_vtx[k].y);
			min_bpt.x = min(min_bpt.x,pt.x);
			min_bpt.y = min(min_bpt.y,pt.y);
			max_bpt.x = max(max_bpt.x,pt.x);
			max_bpt.y = max(max_bpt.y,pt.y);				
		}

		bool cross_FOV = (min_bpt.x < cross_p) || (min_bpt.y < cross_p) || (max_bpt.x > r_img->width-cross_p) || (max_bpt.y > r_img->height-cross_p);

		/**/
		if ((cross_FOV) && constraint[4])
			continue;

		if (fabs(theta) < th_th)
			continue;

		if ((eccent > ecc_range[1]) || (eccent < ecc_range[0]))
			continue;

		/* show region r */
		for ( int j = 0; j < r->total; j++ ) {
			CvPoint* pt = CV_GET_SEQ_ELEM( CvPoint, r, j );
			rsptr[pt->x*3+pt->y*c_img->widthStep] = bcolors[i%9][2];
			rsptr[pt->x*3+1+pt->y*c_img->widthStep] = bcolors[i%9][1];
			rsptr[pt->x*3+2+pt->y*c_img->widthStep] = bcolors[i%9][0];

			seg_img->imageData[pt->x+pt->y*seg_img->widthStep] = 255;
		}

		box_xy.x += cvRound(radius)*2/*(box_vtx[1].x-box_vtx[0].x)*/;
		box_xy.y += cvRound(radius)*2/*(box_vtx[2].y-box_vtx[1].y)*/;

		/* draw box & circle */
		icenter.x = cvRound(center.x);
		icenter.y = cvRound(center.y);

		if (debug_image) {
			cvDrawRect(c_img, cvPoint(icenter.x - cvRound(radius), icenter.y - cvRound(radius)), 
			cvPoint(icenter.x + cvRound(radius), icenter.y + cvRound(radius)), CV_RGB(0,255,0),1,CV_AA,0);
			cvCircle( c_img, icenter, cvRound(radius), CV_RGB(255, 255, 0), 1, CV_AA, 0 );
		}

		CvRect rect = cvRect( icenter.x - cvRound(radius), icenter.y - cvRound(radius),
			cvRound(radius)*2, cvRound(radius)*2 );

		cvSeqPush( residual_MSER, &rect );
	}

	if ( residual_MSER->total ) {	/* if # of meaningful MSER is greater than 0 */
		box_xy.x /= residual_MSER->total;
		box_xy.y /= residual_MSER->total;

		box_xy.x = (int)((float)box_xy.x*zoom_x);
		box_xy.y = (int)((float)box_xy.y*zoom_y);

		float min_area = 1e12, max_area = -1.0f;
		int min_indx, max_indx;

		for (int j=0; j<residual_MSER->total; j++) {
			CvRect *rect = (CvRect*)cvGetSeqElem( residual_MSER, j );

			float c_area = rect->width * rect->height;

			if (min_area > c_area) {
				min_area = c_area;
				min_indx = j;
			}

			if (max_area < c_area) {
				max_area = c_area;
				max_indx = j;
			}
		}

		CvRect *srect = (CvRect*)cvGetSeqElem( residual_MSER, min_indx );

		box_xy.x = (float)srect->width;
		box_xy.y = (float)srect->height;
	}

	if (debug_image) {
		sprintf( user_str, "%s%s", filepath2, debugimg );
		cvSaveImage( user_str, c_img );	
	}

	CvPoint min_loc, max_loc;
	double min_val, max_val;
	IplImage *mm_img, *sts_img, *mth_img, *ss_img;

	mm_img = cvCreateImage( cvSize( seg_img->width, seg_img->height + seg_img->height/this->number_of_buttons ), 8, 3);
	cvSetImageROI( mm_img, cvRect(0,0,seg_img->width,seg_img->height) );
	cvMerge( seg_img, seg_img, seg_img, 0, mm_img );
	cvResetImageROI( mm_img );

	sts_img = cvCreateImage( cvSize(seg_img->width/this->number_of_buttons, seg_img->height/this->number_of_buttons), 8, 1 );

	if ( residual_MSER->total ) {	/* if # of meaningful MSER is greater than 0 */

		/* template matching */
		for (int j=0; j<this->number_of_buttons; j++) {

			match_number_image( seg_img, st_img[j], box_xy.x, box_xy.y, min_val, min_loc );

			cvResize( st_img[j], sts_img );
			
			cvSetImageROI( mm_img, cvRect(j*seg_img->width/this->number_of_buttons,seg_img->height,seg_img->width/this->number_of_buttons,seg_img->height/this->number_of_buttons) );
			cvMerge( sts_img, sts_img, sts_img, 0, mm_img );
			cvResetImageROI( mm_img );

			cobject[j].ul = cvPoint(min_loc.x, min_loc.y);
			cobject[j].lr = cvPoint(min_loc.x + box_xy.x, min_loc.y + box_xy.y);
			cobject[j].ctr = cvPoint(min_loc.x + box_xy.x/2, min_loc.y + box_xy.y/2);
			cobject[j].w = box_xy.x;
			cobject[j].h = box_xy.y;
			cobject[j].corr = min_val;
		}

		/* initialize clusters */
		int *cluster = new int[this->number_of_buttons];
		int cluster_id = 0;
		CvRect *clRect = new CvRect[this->number_of_buttons];
		for (int j=0; j<this->number_of_buttons; j++) 
			cluster[j] = -1;

		/* forming clusters using overlapped degree between two regions */
		for (int j=0; j<this->number_of_buttons; j++) {
			
			/* only for meaningful candidates :: if corr == 0, that denotes a meaningless matching. */
			if (cobject[j].corr > 0.5f) {
				if (debug_image) {
					cvLine( mm_img, cvPoint(j*seg_img->width/this->number_of_buttons+seg_img->width/(2*this->number_of_buttons),seg_img->height+seg_img->height/(2*this->number_of_buttons)), cobject[j].ctr, CV_RGB(128,128,128), 1 );
					cvDrawRect( mm_img, cobject[j].ul, cobject[j].lr, CV_RGB(128,128,128), 1 );
				}
			} else
				continue;

			if (cluster[j] != -1)	/* cluster ID is already assigned. */
				continue;
			else					/* new cluster ID */
				cluster[j] = cluster_id++;

			for (int k=0; k<this->number_of_buttons; k++) {
				if ( (j != k) && (cobject[k].corr > 0.5f) ) {
					float ovd = overlap_degree( cobject[j], cobject[k] );

					if ( ovd > exp(-1.0f) + 0.15f) {	/* overlapped area */

						if (cluster[k] == -1)
							cluster[k] = cluster[j];
						else
							cluster[j] = cluster[k];
					}
				}
			}
		}

		if (show_dbg) {
			printf( "[cluster] " );
			for (int k=0; k<this->number_of_buttons; k++) {
				printf( "%d", cluster[k]);
			}
			printf("\n");
		}

		/* merge regions into each cluster */
		for (int k=0; k<cluster_id; k++) {
			clRect[k] = cvRect(-1,-1,-1,-1);

			for (int l=0; l<this->number_of_buttons; l++) {
				if (cluster[l] == k) {
					CvRect tmpRect = cobject[l].getRect();

					if ((clRect[k].width == -1) && (clRect[k].height == -1))
						clRect[k] = tmpRect;
					else {
						clRect[k].x = min(clRect[k].x,tmpRect.x);
						clRect[k].y = min(clRect[k].y,tmpRect.y);

						clRect[k].width = max(clRect[k].x+clRect[k].width,tmpRect.x+tmpRect.width) - clRect[k].x;
						clRect[k].height = max(clRect[k].y+clRect[k].height,tmpRect.y+tmpRect.height) - clRect[k].y;
					}
				}
			}				
		}

		/* check for each cluster */
		for (int k=0; k<cluster_id; k++) {

			float l_min_val = -1;
			int min_idx = -1;
			CvPoint l_min_loc;
			int l_w,l_h;
			bool skip_this_cluster = false;

			for (int l=0; l<this->number_of_buttons; l++) {

				if (cluster[l] == k) {

					CvSeq* rr = cvCreateSeq( CV_SEQ_KIND_GENERIC|CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage );
					cvClearSeq( rr );

					for ( int yy=clRect[k].y; yy<clRect[k].y+clRect[k].height; yy++ )
						for ( int xx=clRect[k].x; xx<clRect[k].x+clRect[k].width; xx++ ) {
							if (seg_img->imageData[xx+yy*seg_img->widthStep] != 0)
								cvSeqPush( rr, &cvPoint(xx,yy) );
						}

					if (!rr->total) {
						skip_this_cluster = true;
						break;
					}

					box = cvMinAreaRect2( rr, 0 );
					cvBoxPoints( box, box_vtx );

					float box_min_x = 1e3, box_min_y = 1e3, box_max_x = -1, box_max_y = -1;
					for (int bb=0; bb<4; bb++) {
						box_min_x = min(box_min_x,box_vtx[bb].x);
						box_min_y = min(box_min_y,box_vtx[bb].y);
						box_max_x = max(box_max_x,box_vtx[bb].x);
						box_max_y = max(box_max_y,box_vtx[bb].y);
					}

					int box_w = min(clRect[k].width, (int)box_max_x-(int)box_min_x+1);
					int box_h = min(clRect[k].height, (int)box_max_y-(int)box_min_y+1);

					box_w = (int)((float)box_w*zoom_x);
					box_h = (int)((float)box_h*zoom_y);

					box_w -= (clRect[k].x + box_w > seg_img->width) ? (clRect[k].x + box_w - seg_img->width ) : 0;
					box_h -= (clRect[k].y + box_h > seg_img->height) ? (clRect[k].y + box_h - seg_img->height ) : 0;

					IplImage* cl_img = cvCreateImage( cvSize(box_w,box_h), 8, 1 );
					cvSetImageROI( seg_img, cvRect(clRect[k].x,clRect[k].y,box_w,box_h) );
					cvCopy( seg_img, cl_img );
					cvResetImageROI( seg_img );

					match_number_image( cl_img, st_img[l], box_w, box_h, min_val, min_loc );

					cvReleaseImage( &cl_img );

					min_loc.x += clRect[k].x;
					min_loc.y += clRect[k].y;

					/*float l_ovd = overlap_degree( cvRect(box_min_x,box_min_y,box_w,box_h), 
						cvRect(min_loc.x,min_loc.y,box_w,box_h) );*/

					/* find the best number with minimum corr */
					if (min_val > l_min_val) {
						l_min_val = min_val;
						min_idx = l;
						l_min_loc = min_loc;
						l_w = box_w;
						l_h = box_h;
					}
				}
			}				

			printf("[%d] %.2f \n", min_idx, l_min_val );

			/* reject false positive with 0.7 :: perfect matching == 0.0f */
			if ((l_min_val > 0.8f) && (!skip_this_cluster)) {

				if (debug_image) {
					cvLine( mm_img, 
						cvPoint(min_idx*seg_img->width/this->number_of_buttons 
						+ seg_img->width/(this->number_of_buttons*2),
						seg_img->height+seg_img->height/(this->number_of_buttons*2)), 
						cvPoint(l_min_loc.x+l_w/2,l_min_loc.y+l_h/2), CV_RGB(255,0,0), 2 );
					cvDrawRect( mm_img, cvPoint(l_min_loc.x,l_min_loc.y), cvPoint(l_min_loc.x+l_w,l_min_loc.y+l_h), CV_RGB(255,0,0), 3 );
				}

				position[min_idx][0] = l_min_loc.x;
				position[min_idx][1] = l_min_loc.y;
				position[min_idx][2] = l_min_loc.x+l_w;
				position[min_idx][3] = l_min_loc.y+l_h;
			}
		}

		delete [] clRect;
		delete [] cluster;
	}

	if (debug_image) {
		sprintf( user_str, "%s%s", filepatho, debugimg );
		cvSaveImage( user_str, mm_img );
	}

	cvReleaseImage( &sts_img );
	cvReleaseImage( &mm_img );
	cvReleaseImage( &seg_img );

	cvReleaseImage( &c_img );
	cvReleaseImage( &g_img );

	return position;
}

void CFindButtons::learn_buttons( IplImage* r_img )
{
	IplImage* g_img = cvCreateImage( cvGetSize(r_img), 8, 1 );
	IplImage* c_img = cvCreateImage( cvGetSize(r_img), 8, 3 );

	if (r_img->nChannels == 3) {
		cvCopyImage( r_img, c_img );
		cvConvertImage( r_img, g_img, CV_BGR2GRAY );
	} else {
		cvCopyImage( r_img, g_img );
		cvMerge( r_img, r_img, r_img, 0, c_img );
	}

	/* set MSER parameters */
	cvExtractMSER( g_img, NULL, &contours, storage, 
		cvMSERParams( 1 /* delta */, cvRound(.001*r_img->width*r_img->height) /* min area */, 
		cvRound(.4*r_img->width*r_img->height) /* max area */, .25 /* max variation */, .2 /* min diversity */ ) );

	uchar* rsptr = (uchar*)c_img->imageData;

	IplImage *seg_img = cvCreateImage(cvGetSize(r_img),8,1);
	cvSetZero( seg_img );

	CvPoint box_xy = cvPoint(0,0);
	int count = 0;
	cvClearSeq( residual_MSER );

	for ( int i = contours->total-1; i >= 0; i-- ) {	/* check ith region/blob */
		CvSeq* r = *(CvSeq**)cvGetSeqElem( contours, i );

		/* min enclosing rect & circle */
		box = cvMinAreaRect2( r, 0 );
		cvBoxPoints( box, box_vtx );
		cvMinEnclosingCircle( r, &center, &radius );			
		
		/* moments */
		IplImage *m_img = cvCloneImage( g_img );
		cvSetZero( m_img );
		for ( int j = 0; j < r->total; j++ ) {
			CvPoint* pt = CV_GET_SEQ_ELEM( CvPoint, r, j );
			m_img->imageData[pt->x+pt->y*m_img->width] = 127;
		}

		cvMoments( m_img, &moments, 1 );			
		cvGetHuMoments( &moments, &hu_moments);

		cvReleaseImage( &m_img );

		CvPoint centre_of_mass = cvPoint( moments.m10/moments.m00, 
			moments.m01/moments.m00 );

		CvPoint box_center = cvPoint((cvRound(box_vtx[0].x)+cvRound(box_vtx[2].x))/2,
			(cvRound(box_vtx[0].y)+cvRound(box_vtx[2].y))/2);

		float mu20 = cvGetCentralMoment( &moments, 2, 0 );
		float mu02 = cvGetCentralMoment( &moments, 0, 2 );
		float mu00 = cvGetCentralMoment( &moments, 0, 0 );
		float mu11 = cvGetCentralMoment( &moments, 1, 1 );

		float theta = 0.5*atan2(2.0f*mu11/mu00, mu20/mu00-mu02/mu00)*180.0f/M_PI;
		float lambda1 = (mu20/mu00+mu02/mu00)/2.0f + sqrt(4.0f*mu11/mu00*mu11/mu00+(mu20/mu00-mu02/mu00)*(mu20/mu00-mu02/mu00))/2.0f;
		float lambda2 = (mu20/mu00+mu02/mu00)/2.0f - sqrt(4.0f*mu11/mu00*mu11/mu00+(mu20/mu00-mu02/mu00)*(mu20/mu00-mu02/mu00))/2.0f;
		float eccent = sqrt(1-lambda2/lambda1);

		if (show_dbg)
			printf("[%s:%d] %f %f %.2f %f\n", debugimg, i, hu_moments.hu1, hu_moments.hu2, theta, eccent );

		/**/
		if ((((hu_moments.hu1 < hu1_range[0]) || (hu_moments.hu1 > hu1_range[1])) || ((hu_moments.hu2 < hu2_range[0]) || (hu_moments.hu2 > hu2_range[1]))) && constraint[1])
			continue;

		float cdist = sqrt((float)((box_center.x - centre_of_mass.x)*(box_center.x - centre_of_mass.x) 
			+ (box_center.y - centre_of_mass.y)*(box_center.y - centre_of_mass.y)));

		/**/
		if ((cdist > radius * cdist_r) && constraint[2])
			continue;

		float a_ratio = (sqrt((float)((cvRound(box_vtx[0].x)-cvRound(box_vtx[1].x))*(cvRound(box_vtx[0].x)-cvRound(box_vtx[1].x))
			+(cvRound(box_vtx[0].y)-cvRound(box_vtx[1].y))*(cvRound(box_vtx[0].y)-cvRound(box_vtx[1].y))))*sqrt((float)((cvRound(box_vtx[1].x)-cvRound(box_vtx[2].x))*(cvRound(box_vtx[1].x)-cvRound(box_vtx[2].x))
			+(cvRound(box_vtx[1].y)-cvRound(box_vtx[2].y))*(cvRound(box_vtx[1].y)-cvRound(box_vtx[2].y)))))/(M_PI*radius*radius);

		/**/
		if (((a_ratio < aratio_r[0]) || (a_ratio > aratio_r[1])) && constraint[3])
			continue;

		CvPoint min_bpt = cvPoint(1e3,1e3), max_bpt = cvPoint(-1,-1);
		for ( int k=0; k<4; k++)
		{
			pt.x = cvRound(box_vtx[k].x);
			pt.y = cvRound(box_vtx[k].y);
			min_bpt.x = min(min_bpt.x,pt.x);
			min_bpt.y = min(min_bpt.y,pt.y);
			max_bpt.x = max(max_bpt.x,pt.x);
			max_bpt.y = max(max_bpt.y,pt.y);				
		}

		bool cross_FOV = (min_bpt.x < cross_p) || (min_bpt.y < cross_p) || (max_bpt.x > r_img->width-cross_p) || (max_bpt.y > r_img->height-cross_p);

		/**/
		if ((cross_FOV) && constraint[4])
			continue;

		if ((fabs(theta) < th_th) && constraint[5])
			continue;

		if (((eccent > ecc_range[1]) || (eccent < ecc_range[0])) && constraint[6])
			continue;

		/* show region r */
		for ( int j = 0; j < r->total; j++ ) {
			CvPoint* pt = CV_GET_SEQ_ELEM( CvPoint, r, j );
			rsptr[pt->x*3+pt->y*c_img->widthStep] = bcolors[i%9][2];
			rsptr[pt->x*3+1+pt->y*c_img->widthStep] = bcolors[i%9][1];
			rsptr[pt->x*3+2+pt->y*c_img->widthStep] = bcolors[i%9][0];

			seg_img->imageData[pt->x+pt->y*seg_img->widthStep] = 255;

			//printf("[%d]%d %d\n",j,pt->x,pt->y);
		}

		box_xy.x += cvRound(radius)*2/*(box_vtx[1].x-box_vtx[0].x)*/;
		box_xy.y += cvRound(radius)*2/*(box_vtx[2].y-box_vtx[1].y)*/;

		/* draw box & circle */
		icenter.x = cvRound(center.x);
		icenter.y = cvRound(center.y);

		if (debug_image) {
			//printf("[>> learn_buttons] no. of pixels = %d \n", r->total);
			cvDrawRect(c_img, cvPoint(icenter.x - cvRound(radius), icenter.y - cvRound(radius)), 
			cvPoint(icenter.x + cvRound(radius), icenter.y + cvRound(radius)), CV_RGB(0,255,0),1,CV_AA,0);
			cvCircle( c_img, icenter, cvRound(radius), CV_RGB(255, 255, 0), 1, CV_AA, 0 );
		}

		CvRect rect = cvRect( icenter.x - cvRound(radius), icenter.y - cvRound(radius),
			cvRound(radius)*2, cvRound(radius)*2 );

		cvSeqPush( residual_MSER, &rect );
	}

	if ( residual_MSER->total ) {	/* if # of meaningful MSER is greater than 0 */
		box_xy.x /= residual_MSER->total;
		box_xy.y /= residual_MSER->total;
	}

	if (debug_image) {
		printf("[>> learn_buttons] total no. of MSER = %d\n", residual_MSER->total);
		sprintf( user_str, "%s%s", filepath2, debugimg );
		cvSaveImage( user_str, c_img );	

		sprintf( user_str, "%ss%s", filepath2, debugimg );
		cvSaveImage( user_str, seg_img );
	}

	cvReleaseImage( &c_img );
	cvReleaseImage( &g_img );
}