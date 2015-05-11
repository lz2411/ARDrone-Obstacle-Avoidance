/**
 * @file display_stage.c
 * @author nicolas.brulez@parrot.com
 * @date 2012/09/25
 *
 * This stage is a naive example of how to display video using GTK2 + Cairo
 * In a complete application, all GTK handling (gtk main thread + widgets/window creation)
 *  should NOT be handled by the video pipeline (see the Navigation linux example)
 *
 * The window will be resized according to the picture size, and should not be resized bu the user
 *  as we do not handle any gtk event except the expose-event
 *
 * This example is not intended to be a GTK/Cairo tutorial, it is only an example of how to display
 *  the AR.Drone live video feed. The GTK Thread is started here to improve the example readability
 *  (we have all the gtk-related code in one file)
 */

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>
// Self header file
#include "display_stage.h"
#include "delaunay.h"
 #include <ardrone_tool/UI/ardrone_input.h>
// GTK/Cairo headers
#include <cairo.h>
#include <gtk/gtk.h>

// Funcs pointer definition
const vp_api_stage_funcs_t display_stage_funcs = {
    NULL,
    (vp_api_stage_open_t) display_stage_open,
    (vp_api_stage_transform_t) display_stage_transform,
    (vp_api_stage_close_t) display_stage_close
};

// Extern so we can make the ardrone_tool_exit() function (ardrone_testing_tool.c)
// return TRUE when we close the video window
extern int exit_program;

// Boolean to avoid asking redraw of a not yet created / destroyed window
bool_t gtkRunning = FALSE;



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
    if ( *img != NULL )	return;

    *img = cvCreateImage( size, depth, channels );
    if ( *img == NULL )
    {
        fprintf(stderr, "Error: Couldn't allocate image.  Out of memory?\n");
        exit(-1);
    }
}


// Picture size getter from input buffer size
// This function only works for RGB565 buffers (i.e. 2 bytes per pixel)
static void getPicSizeFromBufferSize (uint32_t bufSize, uint32_t *width, uint32_t *height)
{
    if (NULL == width || NULL == height)
    {
        return;
    }

    switch (bufSize)
    {
    case 50688: //QCIF > 176*144 *2bpp
        *width = 176;
        *height = 144;
        break;
    case 153600: //QVGA > 320*240 *2bpp
        *width = 320;
        *height = 240;
        break;
    case 460800: //360p > 640*360 *2bpp
        *width = 640;
        *height = 360;
        break;
    case 1843200: //720p > 1280*720 *2bpp
        *width = 1280;
        *height = 720;
        break;
    default:
        *width = 0;
        *height = 0;
        break;
    }
}

// Get actual frame size (without padding)
void getActualFrameSize (display_stage_cfg_t *cfg, uint32_t *width, uint32_t *height)
{
    if (NULL == cfg || NULL == width || NULL == height)
    {
        return;
    }

    *width = cfg->decoder_info->width;
    *height = cfg->decoder_info->height;
}

// Redraw function, called by GTK each time we ask for a frame redraw
static gboolean
on_expose_event (GtkWidget *widget,
                 GdkEventExpose *event,
                 gpointer data)
{
    display_stage_cfg_t *cfg = (display_stage_cfg_t *)data;

    if (2.0 != cfg->bpp)
    {
        return FALSE;
    }

    uint32_t width = 0, height = 0, stride = 0;
    getPicSizeFromBufferSize (cfg->fbSize, &width, &height);
    stride = cfg->bpp * width;

    if (0 == stride)
    {
        return FALSE;
    }

    uint32_t actual_width = 0, actual_height = 0;
    getActualFrameSize (cfg, &actual_width, &actual_height);
    gtk_window_resize (GTK_WINDOW (widget), actual_width, actual_height);

    cairo_t *cr = gdk_cairo_create (widget->window);

    cairo_surface_t *surface = cairo_image_surface_create_for_data (cfg->frameBuffer, CAIRO_FORMAT_RGB16_565, width, height, stride);

    cairo_set_source_surface (cr, surface, 0.0, 0.0);

    cairo_paint (cr);

    cairo_surface_destroy (surface);

    cairo_destroy (cr);

    return FALSE;
}

/**
 * Main GTK Thread.
 * On an actual application, this thread should be started from your app main thread, and not from a video stage
 * This thread will handle all GTK-related functions
 */
DEFINE_THREAD_ROUTINE(gtk, data)
{
    GtkWidget *window = gtk_window_new (GTK_WINDOW_TOPLEVEL);

    display_stage_cfg_t *cfg = (display_stage_cfg_t *)data;
    cfg->widget = window;

    g_signal_connect (window, "expose-event", G_CALLBACK (on_expose_event), data);
    g_signal_connect (window, "destroy", G_CALLBACK (gtk_main_quit), NULL);

    gtk_window_set_position (GTK_WINDOW (window), GTK_WIN_POS_CENTER);
    gtk_window_set_default_size (GTK_WINDOW (window), 10, 10);
    gtk_widget_set_app_paintable (window, TRUE);
    gtk_widget_set_double_buffered (window, FALSE);

    gtk_widget_show_all (window);

    gtkRunning = TRUE;

    gtk_main ();

    gtkRunning = FALSE;

    // Force ardrone_tool to close
    exit_program = 0;

    // Sometimes, ardrone_tool might not finish properly
    // This happens mainly because a thread is blocked on a syscall
    // in this case, wait 5 seconds then kill the app
    sleep (5);
    exit (0);

    return (THREAD_RET)0;
}
FILE *f;
C_RESULT display_stage_open (display_stage_cfg_t *cfg, FILE *f)
{   
    //rdrone_tool_set_ui_pad_start(1); 
    // Check that we use RGB565
    if (2 != cfg->bpp)
    {
        // If that's not the case, then don't display anything
        cfg->paramsOK = FALSE;
    }
    else
    {
        // Else, start GTK thread and window
        cfg->paramsOK = TRUE;
        cfg->frameBuffer = NULL;
        cfg->fbSize = 0;
        START_THREAD (gtk, cfg);
        
        printf("check check zero\n");
        f = fopen("local change value", "a");
    }
    return C_OK;
}


IplImage *frame_buffer1 = NULL;
IplImage *frame_buffer2 = NULL;

C_RESULT display_stage_transform (display_stage_cfg_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out, FILE *f)
{

    if (frame_buffer1!= NULL && frame_buffer2!=NULL)
    {
        uint32_t width = 0, height = 0;
        getPicSizeFromBufferSize (in->size, &width, &height);

        cvNamedWindow("video", CV_WINDOW_AUTOSIZE);

        //---------------------Optical Flow --------------------------------------//
        static IplImage *frame = NULL, *frame1 = NULL,*frame1_1C = NULL, *frame2_1C = NULL, *eig_image = NULL,
                *temp_image = NULL, *pyramid1 = NULL, *pyramid2 = NULL;
        frame = frame_buffer1; //first frame
        //frame = ipl_image_from_data((uint8_t*)in->buffers[0], 1, 640, 360); //first frame

        if (frame == NULL)
        {
            fprintf(stderr, "Error: Hmm. The end came sooner than we thought.\n");
            return -1;
        }
        CvSize frame_size; frame_size.height = 360; frame_size.width = 640;
        allocateOnDemand( &frame1_1C, frame_size, IPL_DEPTH_8U, 1 );
        cvConvertImage(frame, frame1_1C, 0); //grep image

        allocateOnDemand( &frame1, frame_size, IPL_DEPTH_8U, 3 );
        cvConvertImage(frame, frame1, 0); //colour image

        frame = frame_buffer2; //second frame
        //frame = ipl_image_from_data((uint8_t*)in->buffers[0], 1, 640, 360); //first frame
        if (frame == NULL)
        {
            fprintf(stderr, "Error: Hmm. The end came sooner than we thought.\n");
            return -1;
        }
        allocateOnDemand( &frame2_1C, frame_size, IPL_DEPTH_8U, 1 );
        cvConvertImage(frame, frame2_1C, 0); //grey image
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
        char optical_flow_found_feature[400];
        float optical_flow_feature_error[400];
        CvSize optical_flow_window = cvSize(30,30);
        CvTermCriteria optical_flow_termination_criteria
            = cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 );

        allocateOnDemand( &pyramid1, frame_size, IPL_DEPTH_8U, 1 );
        allocateOnDemand( &pyramid2, frame_size, IPL_DEPTH_8U, 1 );

        cvCalcOpticalFlowPyrLK(frame1_1C, frame2_1C, pyramid1, pyramid2, frame1_features, frame2_features, number_of_features,
                               optical_flow_window, 5, optical_flow_found_feature, optical_flow_feature_error, optical_flow_termination_criteria, 0 );
        int i;
        //int left_sum =0, middle_sum=0, right_sum=0, down_sum=0, up_sum=0;
        for(i = 0; i < number_of_features; i++) //drawing vector flow
        {
            if ( optical_flow_found_feature[i] == 0 )	continue;

            int line_thickness;				line_thickness = 1;

            CvScalar line_color;			line_color = CV_RGB(0,255,0);

            CvPoint p,q;
            p.x = (int) frame1_features[i].x;
            p.y = (int) frame1_features[i].y;
            q.x = (int) frame2_features[i].x;
            q.y = (int) frame2_features[i].y;

            // Balance Strategy
            // double distance;  distance = sqrt( (p.y - q.y)*(p.y - q.y) + (p.x - q.x)*(p.x - q.x) );
 
            // if((p.x <= frame_size.width/3)&&(p.x >0)){
            //     left_sum += distance;
            // }
            // else if((p.x <= frame_size.width*2/3)&&(p.x > frame_size.width/3)){
            //     middle_sum += distance;
            // }
            // else if((p.x <= frame_size.width)&&(p.x >frame_size.width*2/3)){
            //     right_sum += distance;
            // }
            // if((p.y >0)&&(p.y <= frame_size.height/2)){
            //     up_sum += distance;
            // }
            // else if((p.y > frame_size.height/2)&&(p.y <= frame_size.height)){
            //     down_sum += distance;
            // }

            double angle;		angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
            double hypotenuse;	hypotenuse = sqrt( (p.y - q.y)*(p.y - q.y) + (p.x - q.x)*(p.x - q.x) );

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
        // // --------------------Balance Strategy------------------------------//
        // char decision[20] ;
        // if (up_sum > down_sum)
        //     strcpy(decision, "go down");
        //     //printf("go down  ");
        // else if(up_sum < down_sum)
        //     //printf("go up  ");
        //     strcpy(decision, "go up");        
        // else 
        //     strcpy(decision, "stay middle");//printf("stay middle  ");

        // char decision1[20];
        // if((left_sum > middle_sum)&&(middle_sum> right_sum))
        //     strcpy(decision1,"go right");//printf("go right \n");
        // if((left_sum > middle_sum)&&(right_sum > middle_sum))
        //     strcpy(decision1, "go forward");//printf("go forward \n");
        // if ((middle_sum>left_sum)&&(right_sum>left_sum))
        //     strcpy(decision1, "go left");//printf("go left \n");

        // char text [40]; char text1[30]; char text2[40];
        // CvPoint coord;   CvPoint coord1;   CvPoint coord2; 
        // coord.x = coord1.x = coord2.x = 70; 
        // coord.y = 290; coord1.y = 320; coord2.y = 350 ; 
        // CvFont font; CvScalar text_color; text_color = CV_RGB(255,0,0);
        // cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX , 1.0f, 1.0f, 0, 2, 8 );
        // sprintf(text, "Left: %d middle: %d right: %d", left_sum, middle_sum, right_sum); 
        // sprintf(text1, "Up: %d  Down: %d", up_sum,down_sum); 
        // sprintf(text2, "Decision: %s and %s", decision, decision1); 
        // cvPutText(frame1, text, coord, &font, text_color);
        // cvPutText(frame1, text1, coord1, &font, text_color);
        // cvPutText(frame1, text2, coord2, &font, text_color); 
        

        //----------------Delaunay Trianglation-----------------------------//
        CvSubdiv2D* subdiv_1; CvSubdiv2D* subdiv2_1 ; CvSubdiv2D* subdiv3_1;
        CvSubdiv2D* subdiv_2; CvSubdiv2D* subdiv2_2 ; CvSubdiv2D* subdiv3_2;
        CvMemStorage* storage;
        CvRect rect = { 0, 0, 640, 360 };  
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
                // if(s_l[k] > 1)
                //     printf("%.2f, %.2f,,, ", d1_l,d2_l);
            //printf("%.2f, %.2f, %.2f,,, ", d2_l,d2_m,d2_r);
                //cvShowImage("video", frame1);
                //cvWaitKey(0);
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
        //median_l = median(s_l, (sizeof (s_l) /sizeof (s_l[0])),l_zero,f);
        //median_m = median(s_m, (sizeof (s_m) /sizeof (s_m[0])),m_zero,f);
        //median_r = median(s_r, (sizeof (s_r) /sizeof (s_r[0])),r_zero, f);
        
        //fprintf(f, "The median of the array was ");  //////segmentatio fault????  

        //printf(" %0.5f,%0.5f,%0.5f \n",median_l, median_m, median_r);
        //int toc_l,toc_m, toc_r;
        //toc_l = 1.0/median_l; toc_m = 1.0/median_m; toc_r = 1.0/median_r; 
        // if((median_l > 1)||(median_m>1)||(median_r>1))
        //      printf(" %d,%d,%d \n",toc_l, toc_m, toc_r);
        char decision1[20];
        if((median_l>0.02)&&(median_l > median_r)&&(median_l > median_m))
            strcpy(decision1,"attention left");
        else if((median_r>0.02)&&(median_r > median_l)&&(median_r > median_m))
            strcpy(decision1, "attention right");
        else if((median_m>0.02)&&(median_m > median_l)&&(median_m > median_r))
            strcpy(decision1, "attention middle");
        else
            strcpy(decision1, "");
        //printf(" %s \n",decision1);
        char text [100]; char text2[40];
        CvPoint coord;    CvPoint coord2; 
        coord.x  = coord2.x = 40; 
        coord.y = 290;  coord2.y = 350 ; 
        CvFont font; CvScalar text_color; text_color = CV_RGB(255,0,0);
        cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX , 1.0f, 1.0f, 0, 2, 8 );
        sprintf(text, "Left: %0.4f middle: %0.4f right: %0.4f", median_l, median_m, median_r); 
        // //sprintf(text1, "Up: %d  Down: %d", up_sum,down_sum); 
        sprintf(text2, "Decision: %s ", decision1); 
        //cvPutText(frame1, text, coord, &font, text_color);
        // //cvPutText(frame1, text1, coord1, &font, text_color);
        //cvPutText(frame1, text2, coord2, &font, text_color); 

        cvShowImage("video", frame1);
        cvWaitKey(1);
        cvReleaseMemStorage( &storage );
        cvReleaseImage(&frame1);
        frame_buffer1 = frame_buffer2;
        frame_buffer2 = ipl_image_from_data((uint8_t*)in->buffers[0], 1, 640, 360);

    }

    else if(frame_buffer1!=NULL && frame_buffer2==NULL){
        frame_buffer2 = ipl_image_from_data((uint8_t*)in->buffers[0], 1, 640, 360);
        //printf("second frame! \n");

    }
    else if(frame_buffer1 == NULL && frame_buffer2 == NULL){
        frame_buffer1 = ipl_image_from_data((uint8_t*)in->buffers[0], 1, 640, 360);
        frame_buffer2 = NULL;
        //printf("first frame! \n");

    }
    else if(frame_buffer1 == frame_buffer2){
        printf("frames are the same \n");
    }
    return C_OK;

}

C_RESULT display_stage_close (display_stage_cfg_t *cfg,FILE *f)
{   //ardrone_tool_set_ui_pad_select(1);
    //ardrone_tool_set_ui_pad_start(0); 
    // Free all allocated memory
    if (NULL != cfg->frameBuffer)
    {
        vp_os_free (cfg->frameBuffer);
        cfg->frameBuffer = NULL;
        fclose(f);
    }
    return C_OK;
}
