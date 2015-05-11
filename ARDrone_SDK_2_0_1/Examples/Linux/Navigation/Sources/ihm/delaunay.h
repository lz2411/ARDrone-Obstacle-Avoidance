#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <stdio.h>   
#include <math.h>

static CvSubdiv2D* init_delaunay( CvMemStorage* storage,
                           CvRect rect )
{
    CvSubdiv2D* subdiv;

    subdiv = cvCreateSubdiv2D( CV_SEQ_KIND_SUBDIV2D, sizeof(*subdiv),
                               sizeof(CvSubdiv2DPoint),
                               sizeof(CvQuadEdge2D),
                               storage );
    cvInitSubdivDelaunay2D( subdiv, rect );

    return subdiv;
}
// static void draw_subdiv_point( IplImage* img, CvPoint2D32f fp, CvScalar color )
// {
//     cvCircle( img, cvPoint(cvRound(fp.x), cvRound(fp.y)), 3, color, CV_FILLED, 8, 0 );
// }


// static void draw_subdiv_edge( IplImage* img, CvSubdiv2DEdge edge, CvScalar color )
// {
//     CvSubdiv2DPoint* org_pt;
//     CvSubdiv2DPoint* dst_pt;
//     CvPoint2D32f org;
//     CvPoint2D32f dst;
//     CvPoint iorg, idst;

//     org_pt = cvSubdiv2DEdgeOrg(edge);
//     dst_pt = cvSubdiv2DEdgeDst(edge);

//     if( org_pt && dst_pt )
//     {
//         org = org_pt->pt;
//         dst = dst_pt->pt;

//         iorg = cvPoint( cvRound( org.x ), cvRound( org.y ));
//         idst = cvPoint( cvRound( dst.x ), cvRound( dst.y ));

//         cvLine( img, iorg, idst, color, 1, CV_AA, 0 );
//     }
// }


// static void draw_subdiv( IplImage* img, CvSubdiv2D* subdiv,
//                   CvScalar delaunay_color, CvScalar voronoi_color )
// {
//     CvSeqReader  reader;
//     int i, total = subdiv->edges->total;
//     int elem_size = subdiv->edges->elem_size;

//     cvStartReadSeq( (CvSeq*)(subdiv->edges), &reader, 0 );

//     for( i = 0; i < total; i++ )
//     {
//         CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr);

//         if( CV_IS_SET_ELEM( edge ))
//         {
//             //draw_subdiv_edge( img, (CvSubdiv2DEdge)edge + 1, voronoi_color );   //draw voroni cell 
//             draw_subdiv_edge( img, (CvSubdiv2DEdge)edge, delaunay_color );
//         }

//         CV_NEXT_SEQ_ELEM( elem_size, reader );
//     }
// }


static void locate_point( CvSubdiv2D* subdiv, CvPoint2D32f fp, IplImage* img,
                   CvScalar active_color )
{
    CvSubdiv2DEdge e;
    CvSubdiv2DEdge e0 = 0;
    CvSubdiv2DPoint* p = 0;

    cvSubdiv2DLocate( subdiv, fp, &e0, &p );

    if( e0 )
    {
        e = e0;
        do
        {
            //draw_subdiv_edge( img, e, active_color );   //debug purpose 
            e = cvSubdiv2DGetEdge(e,CV_NEXT_AROUND_LEFT);
        }
        while( e != e0 );
    }

    //draw_subdiv_point( img, fp, active_color );   //debug purpose 
}


// static void draw_subdiv_facet( IplImage* img, CvSubdiv2DEdge edge )
// {
//     CvSubdiv2DEdge t = edge;
//     int i, count = 0;
//     CvPoint* buf = 0;

//     // count number of edges in facet
//     do
//     {
//         count++;
//         t = cvSubdiv2DGetEdge( t, CV_NEXT_AROUND_LEFT );
//     } while (t != edge );

//     buf = (CvPoint*)vp_os_malloc( count * sizeof(buf[0]));

//     // gather points
//     t = edge;
//     for( i = 0; i < count; i++ )
//     {
//         CvSubdiv2DPoint* pt = cvSubdiv2DEdgeOrg( t );
//         if( !pt ) break;
//         buf[i] = cvPoint( cvRound(pt->pt.x), cvRound(pt->pt.y));
//         t = cvSubdiv2DGetEdge( t, CV_NEXT_AROUND_LEFT );
//     }

//     if( i == count )
//     {
//         CvSubdiv2DPoint* pt = cvSubdiv2DEdgeDst( cvSubdiv2DRotateEdge( edge, 1 ));
//         cvFillConvexPoly( img, buf, count, CV_RGB(rand()&255,rand()&255,rand()&255), CV_AA, 0 );
//         cvPolyLine( img, &buf, &count, 1, 1, CV_RGB(0,0,0), 1, CV_AA, 0);
//         draw_subdiv_point( img, pt->pt, CV_RGB(0,0,0));
//     }
//     vp_os_free( buf );
// }

// static void paint_voronoi( CvSubdiv2D* subdiv, IplImage* img )
// {
//     CvSeqReader  reader;
//     int i, total = subdiv->edges->total;
//     int elem_size = subdiv->edges->elem_size;

//     cvCalcSubdivVoronoi2D( subdiv );

//     cvStartReadSeq( (CvSeq*)(subdiv->edges), &reader, 0 );

//     for( i = 0; i < total; i++ )
//     {
//         CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr);

//         if( CV_IS_SET_ELEM( edge ))
//         {
//             CvSubdiv2DEdge e = (CvSubdiv2DEdge)edge;
//             // left
//             draw_subdiv_facet( img, cvSubdiv2DRotateEdge( e, 1 ));

//             // right
//             draw_subdiv_facet( img, cvSubdiv2DRotateEdge( e, 3 ));
//         }

//         CV_NEXT_SEQ_ELEM( elem_size, reader );
//     }
// }
static void find_subdiv_three_part(CvSubdiv2D* subdiv, CvSubdiv2D* subdiv2 , CvSubdiv2D* subdiv3,int number_of_features,
                                    CvPoint2D32f* frame_features, int frame_width,int frame_height, IplImage* frame ){
        int j;
        //CvRect rect = { 0, 0, 640, 360 };  //unsure
        //CvMemStorage* storage;
        //CvSubdiv2D* subdiv; CvSubdiv2D* subdiv2 ; CvSubdiv2D* subdiv3;
        CvScalar active_facet_color;//, delaunay_color, voronoi_color; //bkgnd_color;
        active_facet_color = CV_RGB( 255, 0, 0 );
        //delaunay_color  = CV_RGB( 0,0,0);
        //voronoi_color = CV_RGB(0, 180, 0);
        //bkgnd_color = CV_RGB(255,255,255);
        //storage = cvCreateMemStorage(0);  
        //subdiv = init_delaunay( storage, rect );
        //subdiv2 = init_delaunay( storage, rect );
        //subdiv3 = init_delaunay( storage, rect );

        for( j = 0; j < number_of_features; j++ ) //(sizeof(frame1_features)/sizeof(frame1_features[0]))
        {   

            CvPoint2D32f fp = frame_features[j]; //frame2 features might be out of range than frame1 
            if((fp.x >= 0.0)&&(fp.y >= 0.0)&&(fp.x <= frame_width)&&(fp.y <=frame_height)){ 
                if(fp.x <= frame_width/3){
                    locate_point( subdiv, fp, frame, active_facet_color ); 
                    //printf("check one\n");
                    //cvShowImage("video", frame);
                    //if( cvWaitKey( 50 ) >= 0 )
                    //break;

                    cvSubdivDelaunay2DInsert( subdiv, fp );
                    //cvCalcSubdivVoronoi2D( subdiv );
                    //draw_subdiv( frame, subdiv, delaunay_color, voronoi_color );   //debug purpose 
                    
                    //cvShowImage( "video", frame );
                    //if( cvWaitKey( 50 ) >= 0 )
                        //break;
                }
                else if((fp.x <= frame_width *2/3) && (fp.x > frame_width/3)){ 
                    locate_point( subdiv2, fp, frame, active_facet_color ); 
                    cvSubdivDelaunay2DInsert( subdiv2, fp );
                    //cvCalcSubdivVoronoi2D( subdiv2 );     
                    //draw_subdiv( frame, subdiv2, delaunay_color, voronoi_color );   //debug purpose
                    //printf("draw done\n");
                }
                else if((fp.x <= frame_width) && (fp.x > frame_width*2/3)){
                    locate_point( subdiv3, fp, frame, active_facet_color ); 
                    cvSubdivDelaunay2DInsert( subdiv3, fp );
                    //cvCalcSubdivVoronoi2D( subdiv3 );
                    //draw_subdiv( frame, subdiv3, delaunay_color, voronoi_color );   //debug purpose 
                }
            }
        }
        //paint_voronoi(subdiv, frame1);    //debug purpose 
}

static double distance_to_nearby_points( CvSubdiv2D* subdiv, CvPoint2D32f fp, IplImage* img, CvScalar active_color)
{
    CvSubdiv2DEdge e;
    CvSubdiv2DEdge e0 = 0;
    CvSubdiv2DPoint* p ;  
    //CvSubdiv2DEdge* first_e = 0;
    cvSubdiv2DLocate( subdiv, fp, &e0, &p ); //coincide so return double pointer vertex p 
    //printf("point is %f,%f\n", fp.x, fp.y);
    //printf("vertex is %f,%f\n", p->pt.x, p->pt.y);
                    //printf("vertex is %d\n", p->id);  return -1??? 
                    //cvSubdiv2DGetVertex (p, &first_e); 
                    //if(p->first ==NULL){printf("no first edge "); }
                    //if(e0 ==0){printf("no e0 "); }
                        //CvSubdiv2DPoint* org_pt;
                        //CvPoint2D32f haha;
                        //haha =  cvSubdiv2DEdgeOrg(p->first)->pt;
                        //org_pt =  cvSubdiv2DEdgeOrg(e0);
                        //haha = org_pt->pt;
                        //float hehe; hehe = (float)haha.x; 
                        //printf("%f\n",hehe);
    e0 = subdiv->recent_edge;
    //printf("check one\n");
    CvPoint2D32f org;
    CvPoint2D32f dst;
    double distance =0.0; 

    if(e0) //first edge of vertex
    {   //printf("e0 exist \n");
        e = e0;
        //draw_subdiv_edge( img, e, active_color );
        do
        {
            //draw_subdiv_edge( img, e, active_color );   //debug purpose 
                                             //this is not right
            org = cvSubdiv2DEdgeOrg(e)->pt;
            dst = cvSubdiv2DEdgeDst(e)->pt;
            if((org.x <= 640)&& (org.y <=360 ) && (dst.x <= 640)&&(dst.y <=360)
                &&(org.x >=0)&& (org.y >= 0 ) && (dst.x >=0)&&(dst.y >=0))
                distance += sqrt( (org.y - dst.y)*(org.y - dst.y) + (org.x - dst.x)*(org.x - dst.x) );

            e = cvSubdiv2DGetEdge(e,CV_NEXT_AROUND_DST); //MAYBE NOT 100% RIGHT

            //printf(" %.2f,%.2f \n",org.x,dst.x); 
        }
        while( e != e0 );
    }
    
    //printf("distance is %.2f, ",distance); 
    //CvScalar point_color = CV_RGB(0, 0, 255);
    //draw_subdiv_point( img, fp, point_color );   //debug purpose 
    return distance; 
}



void swap(double *x, double *y) { 
    double temp = *x; 
    *x = *y; 
    *y = temp; 
} 
void sort(double* arr, int n) { 
/* i'll use bubblesort even though its not efficient */ 
    int sorted = 0; 
    while(sorted==0) { 
    sorted = 1; int i; 
    for(i=0; i<n-1; i++) { 
        if (arr[i] > arr[i+1] ) { 
            swap (arr+i, arr+i+1); /*passing the addresses to swap */ 
            sorted=0; 
        } 
        } 
    } 
}
double findMedian(double* arr, int n, int zeros) { 
    if ((n-zeros)%2==1) /* if there is an odd number of elements I want n/2 - 1 */ 
        //return arr[(n-zeros)/2 + zeros]; 
        return arr[(n-zeros)/2]; 
    else
        //return (arr[(n-zeros)/2 +zeros ]+ arr[(n-zeros)/2 + zeros-1])/2.0; 
        return (arr[(n-zeros)/2]+ arr[(n-zeros)/2-1])/2.0; 
} 



double median(double* arr , int SIZE, int zeros){
    if(SIZE == zeros) //when the array is all 1, means moving away 
        return 0.0; 
    sort(arr, SIZE); 
    float result = findMedian(arr, SIZE, zeros); 
    return result; 
} 

void OverlayImage(IplImage* src, IplImage* overlay, CvPoint location, CvScalar S, CvScalar D)
{   
    int x,y,i;
    printf("pass\n");
    for(x=0;x < overlay->width;x++)
    {
        if(x + location.x >= src->width) continue;
        for(y=0;y < overlay->height;y++)
        {
            if(y + location.y >= src->height) continue; 
                       
            CvScalar source = cvGet2D(src, y+location.y, x+location.x);
            CvScalar over = cvGet2D(overlay, y, x);
            CvScalar merged;
            for(i=0;i<4;i++)
                merged.val[i] = (S.val[i]*source.val[i]+D.val[i]*over.val[i]);
            cvSet2D(src, y+location.y, x+location.x, merged);
        }
    }
}