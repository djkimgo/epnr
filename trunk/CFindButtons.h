#ifndef find_buttons_h
#define find_buttons_h

#include <cv.h>

static CvScalar colors[] = 
{
    {{0,0,255}},
    {{0,128,255}},
    {{0,255,255}},
    {{0,255,0}},
    {{255,128,0}},
    {{255,255,0}},
    {{255,0,0}},
    {{255,0,255}},
    {{255,255,255}},
{{196,255,255}},
{{255,255,196}}
};

static uchar bcolors[][3] = 
{
    {0,0,255},
    {0,128,255},
    {0,255,255},
    {0,255,0},
    {255,128,0},
    {255,255,0},
    {255,0,0},
    {255,0,255},
    {255,255,255}
};

class CObject {
public:
	CvPoint ul, lr, ctr;
	int w,h,id;
	double hu, eccent, corr;

	CvRect CObject::getRect() { return cvRect(ul.x,ul.y,w,h); }
};

class CFindButtons {
public:
	char basepath[256];
	char user_str[256];
	char filepath[256];
	char filepath2[256];
	char filepath3[256];
	char filepathx[256];
	char filepatht[256];
	char filepatho[256];
	char debugimg[256];

	float hu1_range[2], hu2_range[2];
	float cdist_r;
	float aratio_r[2];
	int cross_p;
	bool constraint[10];
	bool debug_image;
	bool show_dbg;
	double objectInfo[10][6];
	float th_th;
	float ecc_range[2];
	int resize_num;

	CvSeq* contours;
	CvSeq* residual_MSER;
	CvSeq* minSeq;
	CvSeq* objSeq;
	CvMemStorage* storage;

	CvPoint pt0, pt;
    CvBox2D box;
    CvPoint2D32f box_vtx[4];
    CvPoint2D32f center;
    CvPoint icenter;
    float radius;
	float zoom_x, zoom_y;
	CvMoments moments;
	CvHuMoments hu_moments;

	IplImage** t_img;
	IplImage** st_img;
	IplImage* t_img_tiny;

	int number_of_buttons;
	int type_buttons;
	//CObject *cobject;

	CFindButtons();
	~CFindButtons();

	void set_basepath( char * );
	void load_templates();
	void set_debug_info( char * );
	void resize_factor( int a ) { resize_num = a; }
	void set_type_buttons( int a ) { type_buttons = a; }
	void set_zoom_factor( float a, float b) { zoom_x = a; zoom_y = b; }

	void debug_on() { debug_image = true; }
	void debug_off() { debug_image = false; }
	void show_debug() { show_dbg = true; }
	void hide_debug() { show_dbg = false; }

	void set_cdist_r( float a ) { cdist_r = a;}
	void set_cross_p( int a ) { cross_p = a;}
	void set_aratio_r( float *a ) { for (int j=0; j<2; j++) aratio_r[j] = a[j]; }
	void set_hu1_range( float *a ) { for (int j=0; j<2; j++) hu1_range[j] = a[j]; }
	void set_hu2_range( float *a ) { for (int j=0; j<2; j++) hu2_range[j] = a[j]; }

	int** detect_buttons( IplImage* img );
	void learn_buttons( IplImage* img );
	float overlap_degree( CObject , CObject  );
	float overlap_degree( CvRect , CvRect  );
	void match_number_image( const IplImage* , const IplImage* , const int , const int , CvSeq *, int  );
};

#endif