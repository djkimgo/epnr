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
	hu1_range[0] = 2e-3; hu1_range[1] = 1e-3;
	hu2_range[0] = 1e-5; hu2_range[1] = 1e-5;
	cdist_r = 0.2f;
	aratio_r[0] = 0.5f; aratio_r[1] = 0.35f;
	cross_p = 5;
	th_th = 60.0f;
	ecc_range[0] = 0.76f;
	ecc_range[1] = 0.98f;
	/* constraint list : 
	1. overlap with SURF points
	2. Hu moments hu1<1e-3 hu1>2e-3 hu2>1e-5
	3. |box_center - centre_of_mass| > r * 0.2
	4. area_ratio (area_box/area_circle) < 0.3
	5. cross_FOV w/ 5 pixels
	6. 
	*/
	constraint[0] = false;
	constraint[1] = true;
	constraint[2] = false;
	constraint[3] = true;
	constraint[4] = true;
	constraint[5] = false;
	constraint[6] = false;
	constraint[7] = false;
	constraint[8] = false;
	constraint[9] = false;

	/* template info */
	objectInfo[0][0] = 0.001541f; objectInfo[0][1] = 0.946860f;
	objectInfo[1][0] = 0.001459f; objectInfo[1][1] = 0.814954f;
	objectInfo[2][0] = 0.001318f; objectInfo[2][1] = 0.798589f;
	objectInfo[3][0] = 0.001104f; objectInfo[3][1] = 0.906159f;
	objectInfo[4][0] = 0.001204f; objectInfo[4][1] = 0.768144f;
	objectInfo[5][0] = 0.001496f; objectInfo[5][1] = 0.892861f;
	objectInfo[6][0] = 0.001608f; objectInfo[6][1] = 0.869018f;
	objectInfo[7][0] = 0.001504f; objectInfo[7][1] = 0.966262f;
	objectInfo[8][0] = 0.001160f; objectInfo[8][1] = 0.919971f;
	objectInfo[9][0] = 0.001448f; objectInfo[9][1] = 0.942105f;

	storage = cvCreateMemStorage(0);

	number_of_buttons = 10;
	cobject = new CObject[number_of_buttons];

	residual_MSER = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvRect), storage );

	debug_image = true;
	show_dbg = false;
	resize_num = 2;
}

void CFindButtons::load_templates()
{
	/* template loading */
	t_img = new IplImage*[number_of_buttons];
	st_img = new IplImage*[number_of_buttons];
	for (int i=0; i<number_of_buttons; i++) {
		sprintf( user_str, "%s%d.jpg", filepatht, i+1 );		
		t_img[i] = cvLoadImage( user_str, 0 );
		sprintf( user_str, "%sstest%d.jpg", filepatht, i+1 );		
		st_img[i] = cvLoadImage( user_str, 0 );
	}
	t_img_tiny = cvCreateImage( cvSize(t_img[0]->width/number_of_buttons, t_img[0]->height/number_of_buttons), 8, 1 );
}

CFindButtons::~CFindButtons() 
{	
	if (t_img_tiny)
		cvReleaseImage( &t_img_tiny );

	for (int i=0; i<number_of_buttons; i++) {
		cvReleaseImage( &(st_img[i]) );
		cvReleaseImage( &(t_img[i]) );
	}

	delete [] t_img;
	delete [] st_img;
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
			m_img->imageData[pt->x+pt->y*m_img->width] = 255;
		}

		cvMoments( m_img, &moments, 0 );			
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
		if ((((hu_moments.hu1 > hu1_range[0]) || (hu_moments.hu1 < hu1_range[1])) || (hu_moments.hu2 > hu2_range[0])) && constraint[1])
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
		if (((a_ratio > aratio_r[0]) && (a_ratio < aratio_r[1])) && constraint[3])
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
			if (cobject[j].corr < 1.0f) {
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

			for (int k=0; k<10; k++) {
				if ( (j != k) && (cobject[k].corr < 1.0f) ) {
					float ovd = overlap_degree( cobject[j], cobject[k] );

					if ( ovd > exp(-1.0f)) {	/* overlapped area */

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

			float l_min_val = 1e3;
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

					IplImage* cl_img = cvCreateImage( cvSize(clRect[k].width,clRect[k].height), 8, 1 );
					cvSetImageROI( seg_img, clRect[k] );
					cvCopy( seg_img, cl_img );
					cvResetImageROI( seg_img );

					match_number_image( cl_img, st_img[l], box_w, box_h, min_val, min_loc );

					cvReleaseImage( &cl_img );

					min_loc.x += clRect[k].x;
					min_loc.y += clRect[k].y;

					float l_ovd = overlap_degree( cvRect(box_min_x,box_min_y,box_w,box_h), 
						cvRect(min_loc.x,min_loc.y,box_w,box_h) );

					/* find the best number with minimum corr */
					if (min_val < l_min_val) {
						l_min_val = min_val;
						min_idx = l;
						l_min_loc = min_loc;
						l_w = box_w;
						l_h = box_h;
					}
				}
			}				

			/* reject false positive with 0.7 :: perfect matching == 0.0f */
			if ((l_min_val < 0.7f) && (!skip_this_cluster)) {

				if (debug_image) {
					cvLine( mm_img, cvPoint(min_idx*seg_img->width/10+seg_img->width/20,seg_img->height+seg_img->height/20), cvPoint(l_min_loc.x+l_w/2,l_min_loc.y+l_h/2), CV_RGB(255,0,0), 2 );
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
			m_img->imageData[pt->x+pt->y*m_img->width] = 255;
		}

		cvMoments( m_img, &moments, 0 );			
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
		if ((((hu_moments.hu1 > hu1_range[0]) || (hu_moments.hu1 < hu1_range[1])) || (hu_moments.hu2 > hu2_range[0])) && constraint[1])
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
		if (((a_ratio > aratio_r[0]) && (a_ratio < aratio_r[1])) && constraint[3])
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
	}

	if (debug_image) {
		sprintf( user_str, "%s%s", filepath2, debugimg );
		cvSaveImage( user_str, c_img );	
	}

	cvReleaseImage( &c_img );
	cvReleaseImage( &g_img );
}