/*
 * @ihm_stages_o_gtk.c
 * @author marc-olivier.dzeukou@parrot.com
 * @date 2007/07/27
 *
 * ihm vision thread implementation
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <gtk/gtk.h>
#include <gtk/gtkcontainer.h>
#include <sys/time.h>
#include <time.h>

#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Api/vp_api_stage.h>
#include <VP_Api/vp_api_picture.h>
#include <VP_Stages/vp_stages_io_file.h>
#ifdef USE_ELINUX
#include <VP_Stages/vp_stages_V4L2_i_camif.h>
#else
#include <VP_Stages/vp_stages_i_camif.h>
#endif

#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_delay.h>
#include <VP_Stages/vp_stages_yuv2rgb.h>
#include <VP_Stages/vp_stages_buffer_to_picture.h>

#include <ardrone_tool/Video/video_stage.h>

#ifdef PC_USE_VISION
#include <Vision/vision_draw.h>
#include <Vision/vision_stage.h>
#endif

#include <config.h>


#include <ardrone_tool/ardrone_tool.h>
#include <ardrone_tool/Com/config_com.h>

#include "ihm/ihm.h"
#include "ihm/ihm_vision.h"
#include "ihm/ihm_stages_o_gtk.h"
#include "common/mobile_config.h"
#include "delaunay.h"

#include <video_encapsulation.h>

extern GtkWidget *ihm_ImageWin, *ihm_ImageEntry[9], *ihm_ImageDA, *ihm_VideoStream_VBox;
/* For fullscreen video display */
extern GtkWindow *fullscreen_window;
extern GtkImage *fullscreen_image;
extern GdkScreen *fullscreen;

extern int tab_vision_config_params[10];
extern int vision_config_options;
extern int image_vision_window_view, image_vision_window_status;
extern char video_to_play[16];

static GtkImage *image = NULL;
static GdkPixbuf *pixbuf = NULL;
static GdkPixbuf *pixbuf2 = NULL;

static int32_t pixbuf_width = 0;
static int32_t pixbuf_height = 0;
static int32_t pixbuf_rowstride = 0;
//static uint8_t* pixbuf_data = NULL;

int videoPauseStatus = 0;

float DEBUG_fps = 0.0;

const vp_api_stage_funcs_t vp_stages_output_gtk_funcs = {
    NULL,
    (vp_api_stage_open_t) output_gtk_stage_open,
    (vp_api_stage_transform_t) output_gtk_stage_transform,
    (vp_api_stage_close_t) output_gtk_stage_close
};

/* Widgets defined in other files */
extern GtkWidget * ihm_fullScreenFixedContainer;
extern GtkWidget * ihm_fullScreenHBox;
extern GtkWidget * video_information;

/* Information about the video pipeline stages */
extern video_com_multisocket_config_t icc;

extern parrot_video_encapsulation_codecs_t video_stage_decoder_lastDetectedCodec;

extern float DEBUG_nbSlices;
extern float DEBUG_totalSlices;
extern int DEBUG_missed;
extern float DEBUG_fps; // --> a calculer dans le ihm_stages_o_gtk.c
extern float DEBUG_bitrate;
extern float DEBUG_latency;
extern int DEBUG_isTcp;


C_RESULT output_gtk_stage_open(vp_stages_gtk_config_t *cfg)//, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
    return (SUCCESS);
}

void destroy_image_callback(GtkWidget *widget, gpointer data) {
    image = NULL;
}

char video_information_buffer[1024];
int video_information_buffer_index = 0;

IplImage *ipl_image_from_data(uint8_t* data, int reduced_image, int width, int height)
{
  IplImage *currframe;
  IplImage *dst;

  currframe = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 3);
  dst = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 3);

  currframe->imageData = data;
  //cvSetData(currframe,data,8); //Step is wrong??
  cvCvtColor(currframe, dst, CV_BGR2RGB);
  cvReleaseImage(&currframe);

  return dst;
}
inline static void allocateOnDemand( IplImage **img, CvSize size, int depth, int channels )
{
    if ( *img != NULL ) return;

    *img = cvCreateImage( size, depth, channels );
    if ( *img == NULL )
    {
        fprintf(stderr, "Error: Couldn't allocate image.  Out of memory?\n");
        exit(-1);
    }
}
GdkPixbuf* pixbuf_from_opencv(IplImage *img, int resize)
{
  IplImage* converted = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 3);
  cvCvtColor(img, converted, CV_BGR2RGB);
 
  GdkPixbuf* res = gdk_pixbuf_new_from_data(converted->imageData,
                                           GDK_COLORSPACE_RGB,
                                           FALSE,
                                           8,
                                           converted->width,
                                           converted->height,
                                           converted->widthStep,
                                           NULL,
                                           NULL);
  if (resize)
    res = gdk_pixbuf_scale_simple(res, 320, 240, GDK_INTERP_BILINEAR);
 
  return res;
}
IplImage *frame_buffer1 = NULL;
IplImage *frame_buffer2 = NULL;

C_RESULT output_gtk_stage_transform(vp_stages_gtk_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out) {
    
     if (frame_buffer1!= NULL && frame_buffer2!=NULL)
    {
        //uint32_t width = 0, height = 0;
        //getPicSizeFromBufferSize (in->size, &width, &height);

        //cvNamedWindow("video", CV_WINDOW_AUTOSIZE);
        //printf("check 1");
        static IplImage *frame = NULL, *frame1 = NULL,*frame1_1C = NULL, *frame2_1C = NULL, *eig_image = NULL,
                *temp_image = NULL, *pyramid1 = NULL, *pyramid2 = NULL;
        cvSetImageROI(frame_buffer1, cvRect(160, 0, 320, 360));; //first frame
        frame = frame_buffer1; 
        //frame = ipl_image_from_data((uint8_t*)in->buffers[0], 1, 640, 360); //first frame
        //printf("check2 ");
        if (frame == NULL)
        {
            fprintf(stderr, "Error: Hmm. The end came sooner than we thought.\n");
            return -1;
        }
        CvSize frame_size; frame_size.height = 360; frame_size.width = 320; ///CHANGE HEREEE
        allocateOnDemand( &frame1_1C, frame_size, IPL_DEPTH_8U, 1 );
        cvConvertImage(frame, frame1_1C, 0);

        allocateOnDemand( &frame1, frame_size, IPL_DEPTH_8U, 3 );
        cvConvertImage(frame, frame1, 0);

        cvSetImageROI(frame_buffer2, cvRect(160, 0, 320, 360));;
        frame = frame_buffer2;
        //frame = ipl_image_from_data((uint8_t*)in->buffers[0], 1, 640, 360); //first frame
        if (frame == NULL)
        {
            fprintf(stderr, "Error: Hmm. The end came sooner than we thought.\n");
            return -1;
        }
        allocateOnDemand( &frame2_1C, frame_size, IPL_DEPTH_8U, 1 );
        cvConvertImage(frame, frame2_1C, 0);
        //printf("check3 ");
        allocateOnDemand( &eig_image, frame_size, IPL_DEPTH_32F, 1 );
        allocateOnDemand( &temp_image, frame_size, IPL_DEPTH_32F, 1 );
        /*if(frame2_1C != frame1_1C){
            printf("same data on frame\n");
        }*/
        

        int number_of_features;
        number_of_features = 200;

        CvPoint2D32f frame1_features[number_of_features];
        cvGoodFeaturesToTrack(frame1_1C, eig_image, temp_image, frame1_features, &number_of_features, .01, .01, NULL,3,0,0.04);
        CvPoint2D32f frame2_features[number_of_features];

        char optical_flow_found_feature[number_of_features];

        float optical_flow_feature_error[number_of_features];

        CvSize optical_flow_window = cvSize(30,30);

        CvTermCriteria optical_flow_termination_criteria
            = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 );

        allocateOnDemand( &pyramid1, frame_size, IPL_DEPTH_8U, 1 );
        allocateOnDemand( &pyramid2, frame_size, IPL_DEPTH_8U, 1 );

        cvCalcOpticalFlowPyrLK(frame1_1C, frame2_1C, pyramid1, pyramid2, frame1_features, frame2_features, number_of_features,
                               optical_flow_window, 5, optical_flow_found_feature, optical_flow_feature_error, optical_flow_termination_criteria, 0 );
        int i;
        for(i = 0; i < number_of_features; i++) //drawing vector flow
        {
            if ( optical_flow_found_feature[i] == 0 )   continue;

            int line_thickness;             line_thickness = 1;

            CvScalar line_color;            line_color = CV_RGB(0,255,0);

            CvPoint p,q;
            p.x = (int) frame1_features[i].x;
            p.y = (int) frame1_features[i].y;
            q.x = (int) frame2_features[i].x;
            q.y = (int) frame2_features[i].y;

            double angle;       angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
            double hypotenuse;  hypotenuse = sqrt( (p.y - q.y)*(p.y - q.y) + (p.x - q.x)*(p.x - q.x) );

            q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
            q.y = (int) (p.y - 3 * hypotenuse * sin(angle));

            cvLine( frame1, p, q, line_color, line_thickness, CV_AA, 0 );

            p.x = (int) (q.x + 9 * cos(angle + M_PI/ 4));
            p.y = (int) (q.y + 9 * sin(angle + M_PI / 4));
            cvLine( frame1, p, q, line_color, line_thickness, CV_AA, 0 );
            p.x = (int) (q.x + 9 * cos(angle - M_PI / 4));
            p.y = (int) (q.y + 9 * sin(angle - M_PI / 4));
            cvLine( frame1, p, q, line_color, line_thickness, CV_AA, 0 );
        }

//----------------Delaunay Trianglation-----------------------------//
        CvSubdiv2D* subdiv_1; CvSubdiv2D* subdiv2_1 ; CvSubdiv2D* subdiv3_1;
        CvSubdiv2D* subdiv_2; CvSubdiv2D* subdiv2_2 ; CvSubdiv2D* subdiv3_2;
        CvMemStorage* storage;
        CvRect rect = { 0, 0, 320, 360 };  
        storage = cvCreateMemStorage(0);
        subdiv_1 = init_delaunay( storage, rect );//frame 1 subdivs
        subdiv2_1 = init_delaunay( storage, rect );
        subdiv3_1 = init_delaunay( storage, rect );
        subdiv_2 = init_delaunay( storage, rect ); //frame 2 subdivs
        subdiv2_2 = init_delaunay( storage, rect );
        subdiv3_2 = init_delaunay( storage, rect );
        find_subdiv_three_part(subdiv_1, subdiv2_1 , subdiv3_1,number_of_features, frame1_features, frame_size.width,frame_size.height, frame1);
        find_subdiv_three_part(subdiv_2, subdiv2_2 , subdiv3_2,number_of_features, frame2_features, frame_size.width,frame_size.height, frame2_1C);// is it frame 1?? 
        

        
        //calculate local scale change by finding the sum of distance to immediate neighboors
        int k; 
        double s_l[number_of_features],s_m[number_of_features],s_r[number_of_features]; 
        int l_zero=0, m_zero=0, r_zero=0;
        for( k = 0; k < number_of_features; k++ ){ // for all point in image 
            CvPoint2D32f p,q;   // 2 feature point in frame1 and frame2 
            p =  frame1_features[k];       
            q =  frame2_features[k];
            double d1_l=0,d1_m=0,d1_r=0,d2_l=1,d2_m=1,d2_r=1;
            CvScalar edge_color = CV_RGB( 255, 0, 0 );

            if((p.x >= 0.0)&&(p.y >= 0.0)&&(p.x <= frame_size.width)&&(p.y <=frame_size.height)&&
                (q.x >= 0.0)&&(q.y >= 0.0)&&(q.x <= frame_size.width)&&(q.y <=frame_size.height)){ 
                if((p.x >= 0.0)&&(p.x <= frame_size.width/3)&&(q.x >= 0.0)&&(q.x <= frame_size.width/3))
                {  //on the left side
                    d1_l = distance_to_nearby_points(subdiv_1, p, frame1, edge_color ); //find distance to its neighboors
                    d2_l = distance_to_nearby_points(subdiv_2, q, frame2_1C , edge_color);
                }
                else if((p.x <= frame_size.width *2/3) && (p.x > frame_size.width/3)
                    &&(q.x <= frame_size.width *2/3) && (q.x > frame_size.width/3))
                { //in the middle
                    d1_m = distance_to_nearby_points(subdiv2_1, p, frame1, edge_color );
                    d2_m = distance_to_nearby_points(subdiv2_2, q, frame2_1C, edge_color );
                }
                else if((p.x <= frame_size.width) && (p.x > frame_size.width*2/3)
                    &&(q.x <= frame_size.width) && (q.x > frame_size.width*2/3))
                {
                    d1_r = distance_to_nearby_points(subdiv3_1, p, frame1, edge_color );
                    d2_r = distance_to_nearby_points(subdiv3_2, q, frame2_1C, edge_color );}

                }

             /* local scale change of one frame, when move away, s would be negative and possibly less 
                than -1, and move forward gives s between 0 to 1. we will discard all negative moving away value
                */

                s_l[k] = (d2_l-d1_l)/d2_l;  
                if ((s_l[k] == 1.0)||(s_l[k] < 0.0)){  // 1 is the at the end of the array when sorted
                    s_l[k] = 1.0;
                    l_zero += 1;
                }
                s_m[k] = (d2_m-d1_m)/d2_m; 
                if ((s_m[k] == 1.0)||(s_m[k] < 0.0)){  // 1 is the at the end of the array when sorted
                    s_m[k] = 1.0;
                    m_zero += 1;
                }
                s_r[k]= (d2_r-d1_r)/d2_r;
                //if ((s_r[k] == 1.0)||(s_r[k] < 0.0)){  // 1 is the at the end of the array when sorted
                if ((s_r[k] == 1.0)){
                    //s_r[k] = 1.0;
                    r_zero += 1;
                }
        }
        

        // for (i=0;i < (sizeof (s_l) /sizeof (s_l[0]));i++) {
        //     printf("%.7f  ",s_l[i]);
        //  }
        //  printf(" \n  ");
        //          for (i=0;i < (sizeof (s_m) /sizeof (s_m[0]));i++) {
        //     printf("%.7f  ",s_m[i]);
        //  }
        //  printf(" \n  ");
        // for (i=0;i < (sizeof (s_r) /sizeof (s_r[0]));i++) {
        //     printf("%.5f  ",s_r[i]);
        // }
        // printf(" \n");


        //find median of s_l, s_m, s_r 
        double median_l,median_m,median_r; 
        median_l = median(s_l, (sizeof (s_l) /sizeof (s_l[0])),l_zero);
        median_m = median(s_m, (sizeof (s_m) /sizeof (s_m[0])),m_zero);
        median_r = median(s_r, (sizeof (s_r) /sizeof (s_r[0])),r_zero);

        //fprintf(f, "The median of the array was ");  //////segmentatio fault????  

        //printf(" %0.5f,%0.5f,%0.5f \n",median_l, median_m, median_r);
        //int toc_l,toc_m, toc_r;
        //toc_l = 1.0/median_l; toc_m = 1.0/median_m; toc_r = 1.0/median_r; 
        // if((median_l > 1)||(median_m>1)||(median_r>1))
        //      printf(" %d,%d,%d \n",toc_l, toc_m, toc_r);
        IplImage* imgRed = cvCreateImage(cvSize(frame_size.width *1/3, frame_size.height), 8, 3);
        cvSet(imgRed, CV_RGB(255,0,0),NULL);

        char decision1[20];
        if((median_l>0.02)&&(median_l > median_r)&&(median_l > median_m)){

            OverlayImage(frame1, imgRed, cvPoint(0,0), cvScalar(0.5,0.5,0.5,0.5), cvScalar(0.5,0.5,0.5,0.5));
            strcpy(decision1,"attention left");
            ardrone_at_set_progress_cmd(0,1.0,0.0,0.0,0.0); 
            //cvWaitKey(30);
        }
        else if((median_r>0.02)&&(median_r > median_l)&&(median_r > median_m)){
            OverlayImage(frame1, imgRed, cvPoint(frame_size.width *2/3,0), cvScalar(0.5,0.5,0.5,0.5), cvScalar(0.5,0.5,0.5,0.5));
            strcpy(decision1, "attention right");
            ardrone_at_set_progress_cmd(0,-1.0,0.0,0.0,0.0); 
            //cvWaitKey(30);
        }
        else if((median_m>0.02)&&(median_m > median_l)&&(median_m > median_r)){
            OverlayImage(frame1, imgRed, cvPoint(frame_size.width *1/3,0), cvScalar(0.5,0.5,0.5,0.5), cvScalar(0.5,0.5,0.5,0.5));
            strcpy(decision1, "attention middle");
            ardrone_at_set_progress_cmd(0,1.0,0.0,0.0,0.0);
            //cvWaitKey(30);
        }
        else
            strcpy(decision1, "");
        //printf(" %s \n",decision1);
        char text [100]; char text2[40];
        CvPoint coord;    CvPoint coord2; 
        coord.x  = coord2.x = 10; 
        coord.y = 290;  coord2.y = 350 ; 
        CvFont font; CvScalar text_color; text_color = CV_RGB(255,0,0);
        cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX , 1.0f, 1.0f, 0, 2, 8 );
        sprintf(text, "Left: %0.4f middle: %0.4f right: %0.4f", median_l, median_m, median_r); 
        // //sprintf(text1, "Up: %d  Down: %d", up_sum,down_sum); 
        sprintf(text2, "Decision: %s ", decision1); 
        cvPutText(frame1, text, coord, &font, text_color);
        // //cvPutText(frame1, text1, coord1, &font, text_color);
        cvPutText(frame1, text2, coord2, &font, text_color); 
        cvReleaseMemStorage( &storage );   


        /// then gtk code.....
        if (!ihm_is_initialized) return SUCCESS;
        if (ihm_ImageWin == NULL) return SUCCESS;
        if (image_vision_window_view != WINDOW_VISIBLE) return SUCCESS;

        gdk_threads_enter(); //http://library.gnome.org/devel/gdk/stable/gdk-Threads.html
        static struct timeval tvPrev = {0, 0}, tvNow = {0, 0};
        static int nbFramesForCalc = 1;
    #define CALCULATE_EVERY_X_FRAMES 10
        if (0 == --nbFramesForCalc)
          {
            nbFramesForCalc = CALCULATE_EVERY_X_FRAMES;
            tvPrev.tv_sec = tvNow.tv_sec;
            tvPrev.tv_usec = tvNow.tv_usec;
            gettimeofday(&tvNow, NULL);
            if (0 != tvPrev.tv_sec) // Avoid first time calculation
              {
                float timeDiffMillis = ((tvNow.tv_sec - tvPrev.tv_sec) * 1000.0) + ((tvNow.tv_usec - tvPrev.tv_usec) / 1000.0);
                DEBUG_fps = (0.8 * DEBUG_fps) + (0.2 * ((1000.0 * CALCULATE_EVERY_X_FRAMES) / timeDiffMillis));
              }
          }

        video_decoder_config_t * dec_config;
        dec_config = (video_decoder_config_t *) cfg->last_decoded_frame_info;
        pixbuf_width = dec_config->src_picture->width;
        pixbuf_height = dec_config->src_picture->height;
        pixbuf_rowstride = dec_config->rowstride;
        //pixbuf_data = (uint8_t*) in->buffers[in->indexBuffer];

        if (pixbuf != NULL) {
            g_object_unref(pixbuf);
            pixbuf = NULL;
        }
        pixbuf = pixbuf_from_opencv(frame1,0);
        // pixbuf = gdk_pixbuf_new_from_data(pixbuf_data,
        //     GDK_COLORSPACE_RGB,
        //     FALSE,
        //     8,
        //     pixbuf_width,
        //     pixbuf_height,
        //     pixbuf_rowstride,
        //     NULL,
        //     NULL);

        if (fullscreen != NULL && fullscreen_window != NULL) {
            if (pixbuf2 != NULL) {
                g_object_unref(pixbuf2);
                pixbuf2 = NULL;
            }

            pixbuf2 = gdk_pixbuf_scale_simple(pixbuf,
                gdk_screen_get_width(fullscreen),
                gdk_screen_get_height(fullscreen),
                /*GDK_INTERP_HYPER*/
                cfg->gdk_interpolation_mode);
            /*if (fullscreen_image == NULL)
              {
              fullscreen_image  = (GtkImage*) gtk_image_new_from_pixbuf( pixbuf );
              //if (fullscreen_image == NULL) { printf("Probleme.\n"); }
              //gtk_container_add( GTK_CONTAINER( fullscreen_window ), GTK_WIDGET(fullscreen_image) );
              gtk_fixed_put(ihm_fullScreenFixedContainer,fullscreen_image,0,0);
              }*/
            if (fullscreen_image != NULL) {
                gtk_image_set_from_pixbuf(fullscreen_image, pixbuf2);
                //gtk_widget_show_all (GTK_WIDGET(fullscreen_window));
                gtk_widget_show(GTK_WIDGET(fullscreen_image));
                //gtk_widget_show(ihm_fullScreenHBox);
            }
        } else {

            if (cfg->desired_display_height != 0 && cfg->desired_display_width != 0) /* 0 and 0 means auto mode */ {
                if (pixbuf2 != NULL) {
                    g_object_unref(pixbuf2);
                    pixbuf2 = NULL;
                }

                pixbuf2 = gdk_pixbuf_scale_simple(pixbuf,
                    cfg->desired_display_width,
                    cfg->desired_display_height,
                    cfg->gdk_interpolation_mode);
            } else {
                /* A copy of pixbuf is always made into pixbuf 2.
                  If pixbuf is used directly, GTK renders the video from the buffer allocated by the FFMPEG decoding stage,
                  which becomes invalid when the decoder is resetted (when a codec change occurs for example).
                  This makes GTK crash.
                  TODO : find a reliable way of rendering from the FFMPEG output buffer to avoid the data copy from pixbuf to pixbuf2
                 */

                if (pixbuf2 != NULL) {
                    g_object_unref(pixbuf2);
                    pixbuf2 = NULL;
                }

                pixbuf2 = gdk_pixbuf_copy(pixbuf);
            }

            if (image == NULL && (pixbuf != NULL || pixbuf2 != NULL)) {
                image = (GtkImage*) gtk_image_new_from_pixbuf((pixbuf2) ? (pixbuf2) : (pixbuf));
                gtk_signal_connect(GTK_OBJECT(image), "destroy", G_CALLBACK(destroy_image_callback), NULL);
                if (GTK_IS_WIDGET(ihm_ImageWin))
                    if (GTK_IS_WIDGET(ihm_VideoStream_VBox))
                        gtk_container_add(GTK_CONTAINER(ihm_VideoStream_VBox), (GtkWidget*) image);
            }
            if (image != NULL && (pixbuf != NULL || pixbuf2 != NULL)) {
                if (!videoPauseStatus) gtk_image_set_from_pixbuf(image, (pixbuf2) ? (pixbuf2) : (pixbuf));
            }
        }

        /*---- Display statistics ----*/


        float DEBUG_percentMiss = DEBUG_nbSlices * 100.0 /  DEBUG_totalSlices;


        video_information_buffer_index =
                    snprintf(video_information_buffer,
                        sizeof(video_information_buffer),
                        "%s - %s %dx%d\n",
                        (icc.configs[icc.last_active_socket]->protocol == VP_COM_TCP)?"TCP":(icc.configs[icc.last_active_socket]->protocol == VP_COM_UDP)?"UDP":"?",
                        /*codec*/
                                (video_stage_decoder_lastDetectedCodec == CODEC_MPEG4_AVC )?"H.264":
                                (video_stage_decoder_lastDetectedCodec == CODEC_MPEG4_VISUAL) ? "MP4":
                                (video_stage_decoder_lastDetectedCodec == CODEC_VLIB) ? "VLIB":
                                (video_stage_decoder_lastDetectedCodec == CODEC_P264) ? "P.264": "?",
                                pixbuf_width,pixbuf_height
                        );

        if (video_stage_decoder_lastDetectedCodec == CODEC_MPEG4_AVC )
        {
            video_information_buffer_index+=
                    snprintf(video_information_buffer+video_information_buffer_index,
                            sizeof(video_information_buffer)-video_information_buffer_index,
                            "Mean missed slices :%6.3f/%2.0f (%5.1f%%)\nMissed frames : %10d\nFPS : %4.1f | Bitrate : %6.2f Kbps\nLatency : %5.1f ms | Protocol : %s",\
                                DEBUG_nbSlices,
                                DEBUG_totalSlices,
                                DEBUG_percentMiss,
                                DEBUG_missed,
                                DEBUG_fps,
                                DEBUG_bitrate,
                                DEBUG_latency,
                                (1 == DEBUG_isTcp) ? "TCP" : "UDP");

        }
        else
        {
            video_information_buffer_index+=
                            snprintf(video_information_buffer+video_information_buffer_index,
                                    sizeof(video_information_buffer)-video_information_buffer_index,
                                    "Missed frames : %10d\nFPS : %4.1f | Bitrate : %6.2f Kbps\nLatency : %5.1f ms | Protocol : %s",\
                                        DEBUG_missed,
                                        DEBUG_fps,
                                        DEBUG_bitrate,
                                        DEBUG_latency,
                                        (1 == DEBUG_isTcp) ? "TCP" : "UDP");

        }

        if (video_information){
            gtk_label_set_text((GtkLabel *)video_information,(const gchar*)video_information_buffer);
            gtk_label_set_justify((GtkLabel *)video_information,GTK_JUSTIFY_LEFT);
        }

        gtk_widget_show_all(ihm_ImageWin);
        gdk_threads_leave();


        //return (SUCCESS);

        frame_buffer1 = frame_buffer2;
        frame_buffer2 = ipl_image_from_data((uint8_t*)in->buffers[0], 1, 640, 360);
    }

    else if(frame_buffer1!=NULL && frame_buffer2==NULL){
        frame_buffer2 = ipl_image_from_data((uint8_t*)in->buffers[0], 1, 640, 360);
        //printf("second frame! \n");
        //return C_OK;
    }
    else if(frame_buffer1 == NULL && frame_buffer2 == NULL){
        frame_buffer1 = ipl_image_from_data((uint8_t*)in->buffers[0], 1, 640, 360);
        frame_buffer2 = NULL;
        //printf("first frame! \n");
        //return C_OK;
    }
    else if(frame_buffer1 == frame_buffer2){
        printf("frames are the same \n");
    }

    return (SUCCESS);
}

C_RESULT output_gtk_stage_close(vp_stages_gtk_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out) {
    return (SUCCESS);
}

static vp_os_mutex_t draw_trackers_update;
/*static*/ vp_stages_draw_trackers_config_t draw_trackers_cfg = {0};

/*
void set_draw_trackers_config(vp_stages_draw_trackers_config_t* cfg) {
    void*v;
    vp_os_mutex_lock(&draw_trackers_update);
    v = draw_trackers_cfg.last_decoded_frame_info;
    vp_os_memcpy(&draw_trackers_cfg, cfg, sizeof (draw_trackers_cfg));
    draw_trackers_cfg.last_decoded_frame_info = v;
    vp_os_mutex_unlock(&draw_trackers_update);
}
*/

C_RESULT draw_trackers_stage_open(vp_stages_draw_trackers_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out) {
    vp_os_mutex_lock(&draw_trackers_update);

    int32_t i;
    for (i = 0; i < NUM_MAX_SCREEN_POINTS; i++) {
        cfg->locked[i] = C_OK;
    }

    PRINT("Draw trackers inited with %d trackers\n", cfg->num_points);

    vp_os_mutex_unlock(&draw_trackers_update);

    return (SUCCESS);
}

C_RESULT draw_trackers_stage_transform(vp_stages_draw_trackers_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
    int32_t i;
    video_decoder_config_t * dec_config;
    vp_api_picture_t * picture;
    int pixbuf_width;
    int pixbuf_height;

    dec_config = (video_decoder_config_t *) cfg->last_decoded_frame_info;
    pixbuf_width = dec_config->src_picture->width;
    pixbuf_height = dec_config->src_picture->height;

    vp_os_mutex_lock(&draw_trackers_update);

    picture = dec_config->dst_picture;
    picture->raw = in->buffers[in->indexBuffer];

    if (in->size > 0) {
#if defined DEBUG && 0
        for (i = 0; i < cfg->num_points; i++) {
            int32_t dist;
            uint8_t color;
            screen_point_t point;

            point = cfg->points[i];
            //       point.x += ACQ_WIDTH / 2;
            //       point.y += ACQ_HEIGHT / 2;

            if (point.x >= STREAM_WIDTH || point.x < 0 || point.y >= STREAM_HEIGHT || point.y < 0) {
                PRINT("Bad point (%d,%d) received at index %d on %d points\n", point.x, point.y, i, cfg->num_points);
                continue;
            }

            if (SUCCEED(cfg->locked[i])) {
                dist = 3;
                color = 0;
            } else {
                dist = 1;
                color = 0xFF;
            }

            vision_trace_cross(&point, dist, color, picture);
        }
#endif

        for (i = 0; i < cfg->detected; i++) {
            //uint32_t centerX,centerY;
            uint32_t width, height;
            screen_point_t center;
            if (cfg->last_decoded_frame_info != NULL) {

                center.x = cfg->patch_center[i].x * pixbuf_width / 1000;
                center.y = cfg->patch_center[i].y * pixbuf_height / 1000;
                width = cfg->width[i] * pixbuf_width / 1000;
                height = cfg->height[i] * pixbuf_height / 1000;

                width = min(2 * center.x, width);
                width = min(2 * (pixbuf_width - center.x), width) - 1;
                height = min(2 * center.y, height);
                width = min(2 * (pixbuf_height - center.y), height) - 1;


                trace_reverse_rgb_rectangle(dec_config->dst_picture,center, width, height);

            } else {
                printf("Problem drawing rectangle.\n");
            }
        }
    }

    vp_os_mutex_unlock(&draw_trackers_update);

    out->size = in->size;
    out->indexBuffer = in->indexBuffer;
    out->buffers = in->buffers;

    out->status = VP_API_STATUS_PROCESSING;

    return (SUCCESS);
}

C_RESULT draw_trackers_stage_close(vp_stages_draw_trackers_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out) {
    return (SUCCESS);
}

const vp_api_stage_funcs_t draw_trackers_funcs = {
    NULL,
    (vp_api_stage_open_t) draw_trackers_stage_open,
    (vp_api_stage_transform_t) draw_trackers_stage_transform,
    (vp_api_stage_close_t) draw_trackers_stage_close
};

static inline void reverse(uint8_t * x){
	uint8_t r=*(x);
	uint8_t g=*(x+1);
	uint8_t b=*(x+2);
	*(x)   = r+128;
	*(x+1) = g+128;
	*(x+2) = b+128;
}

void trace_reverse_rgb_h_segment(vp_api_picture_t * picture,int line,int start,int stop)
{
	int i;
	uint8_t *linepointer;
	if (line<0 || line>picture->height-1) return;
	linepointer = &picture->raw[3*picture->width*line];
	for ( i=max(start,0);  i<(picture->width-1) && i<stop ; i++ ) {
          reverse(&linepointer[3*i]);
	};
}

void trace_reverse_rgb_v_segment(vp_api_picture_t * picture,int column,int start,int stop)
{
	int i;
	uint8_t *columnpointer;
	if (column<0 || column>picture->width-1) return;
	columnpointer = &picture->raw[3*(picture->width*start+column)];
	for ( i=max(start,0);  i<(picture->height-1) && i<stop ; i++ ) {
		reverse(&columnpointer[0]);
		columnpointer+=3*picture->width;
	};
}

void trace_reverse_rgb_rectangle( vp_api_picture_t * picture,screen_point_t center, int width, int height)
{

	if (!picture) { return; }
	if (!picture->raw) { printf("NULL pointer\n");return; }
	/*if (PIX_FMT_RGB24!=picture->format) {
		printf("%s:%d - Invalid format : %d/%d\n",__FUNCTION__,__LINE__,PIX_FMT_BGR8,picture->format); return;
	};*/
	trace_reverse_rgb_h_segment(picture,center.y-height/2,center.x-width/2,center.x+width/2);
	trace_reverse_rgb_h_segment(picture,center.y+height/2,center.x-width/2,center.x+width/2);
	trace_reverse_rgb_v_segment(picture,center.x-width/2 ,center.y-height/2,center.y+height/2);
	trace_reverse_rgb_v_segment(picture,center.x+width/2 ,center.y-height/2,center.y+height/2);

	trace_reverse_rgb_h_segment(picture,center.y,center.x-width/4 ,center.x+width/4);
	trace_reverse_rgb_v_segment(picture,center.x,center.y-height/4,center.y+height/4);

}


