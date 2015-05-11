#include <time.h>
#ifndef _WIN32
	#include <sys/time.h>
#else

 #include <sys/timeb.h>
 #include <Winsock2.h>  // for timeval structure

 int gettimeofday (struct timeval *tp, void *tz)
 {
	 struct _timeb timebuffer;
	 _ftime (&timebuffer);
	 tp->tv_sec = (long)timebuffer.time;
	 tp->tv_usec = (long)timebuffer.millitm * 1000;
	 return 0;
 }
#endif

#include <stdlib.h>

#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>

#include <ardrone_tool/ardrone_tool.h>
#include <ardrone_tool/Navdata/ardrone_navdata_file.h>
#include <ardrone_tool/UI/ardrone_input.h>

uint32_t num_picture_decoded = 0;

float32_t nd_iphone_gaz = 0.0;
float32_t nd_iphone_yaw = 0.0;
int32_t nd_iphone_flag = 0;
float32_t nd_iphone_phi = 0.0;
float32_t nd_iphone_theta = 0.0;
float32_t nd_iphone_magneto_psi_accuracy = 0.0;
float32_t nd_iphone_magneto_psi = 0.0;

// Public declaration of navdata_file allowing other handlers to write into
FILE* navdata_file = NULL;

// Private declaration of navdata_file
// Allow this handler to disable other handlers that write in navdata file
static FILE* navdata_file_private = NULL;
static ardrone_navdata_file_data *navdata_file_data = NULL;

static void ardrone_navdata_file_print_version( void )
{
  unsigned int i;
  fprintf(navdata_file,"VERSION 19c\n");  // TODO : CHANGE VERSION NUMBER EVERY TIME THE FILE STRUCTURE CHANGES
  fprintf(navdata_file,
  "Control_state [-]; ARDrone_state [-]; Time [s]; nd_seq_num [-]; \
    AccX_raw [LSB]; AccY_raw [LSB]; AccZ_raw [LSB]; \
    GyroX_raw [LSB]; GyroY_raw [LSB]; GyroZ_raw [LSB];\n");




  #ifdef PC_USE_POLARIS
    fprintf( navdata_file,
    "POLARIS_X [mm]; POLARIS_Y [mm]; POLARIS_Z [mm]; \
  POLARIS_QX [deg]; POLARIS_QY [deg]; POLARIS_QZ [deg];\
  POLARIS_Q0 [deg]; Time s [s]; Time us [us]; ");
  #endif

  #ifdef USE_TABLE_PILOTAGE
      fprintf( navdata_file, " Table_Pilotage_position [mdeg]; Table_Pilotage_vitesse [deg/s]; ");
  #endif

	if((navdata_file_data != NULL) && (navdata_file_data->print_header != NULL))
		navdata_file_data->print_header(navdata_file);
}

struct tm *navdata_atm = NULL;

C_RESULT ardrone_navdata_file_init( void* data )
{
  char filename[1024];
  struct timeval tv;
  time_t temptime;

  navdata_file_data = (ardrone_navdata_file_data*)data;

  gettimeofday(&tv,NULL);
  temptime = (time_t)tv.tv_sec;
  navdata_atm = localtime(&temptime);
  strcpy(filename, root_dir);
  strcat(filename, "/");

  if((navdata_file_data != NULL) && (navdata_file_data->filename != NULL))
  {
	  strcpy(filename, navdata_file_data->filename);
  }
  else
  {
      sprintf(filename, "%s/mesures_%04d%02d%02d_%02d%02d%02d.txt",
        filename,
        navdata_atm->tm_year+1900, navdata_atm->tm_mon+1, navdata_atm->tm_mday,
        navdata_atm->tm_hour, navdata_atm->tm_min, navdata_atm->tm_sec);
  }

  // private for instance
  navdata_file_private = fopen(filename, "wb");

  return navdata_file_private != NULL ? C_OK : C_FAIL;
}

C_RESULT ardrone_navdata_file_process( const navdata_unpacked_t* const pnd )
{
	//uint32_t i;
	char str[50];
	int32_t* locked_ptr;
	screen_point_t* point_ptr;
	struct timeval time;
	input_state_t *input_state = NULL;

	gettimeofday(&time,NULL);

	if( navdata_file_private == NULL )
		return C_FAIL;

	if( ardrone_get_mask_from_state(pnd->ardrone_state, ARDRONE_NAVDATA_BOOTSTRAP) )
		return C_OK;

	if( navdata_file == NULL )
	{
		navdata_file = navdata_file_private;

		if( ardrone_get_mask_from_state(pnd->ardrone_state, ARDRONE_NAVDATA_DEMO_MASK) )
		{
			printf("Receiving navdata demo\n");
		}
		else
		{
			printf("Receiving all navdata\n");
		}
		ardrone_navdata_file_print_version();
	}

	// Handle the case where user asked for a new navdata file
	if( navdata_file != navdata_file_private )
	{
		fclose(navdata_file);
		navdata_file = navdata_file_private;

		if( ardrone_get_mask_from_state(pnd->ardrone_state, ARDRONE_NAVDATA_DEMO_MASK) )
		{
			printf("Receiving navdata demo\n");
		}
		else
		{
			printf("Receiving all navdata\n");
		}
		ardrone_navdata_file_print_version();
	}

	vp_os_memset(&str[0], 0, sizeof(str));
	input_state = ardrone_tool_input_get_state();
	fprintf( navdata_file,"\n" );
	//fprintf( navdata_file, "%u; %u", (unsigned int) pnd->navdata_demo.ctrl_state, (unsigned int) pnd->ardrone_state );

	sprintf( str, "navdata_time %d.%06d", (int)((pnd->navdata_time.time & TSECMASK) >> TSECDEC), (int)(pnd->navdata_time.time & TUSECMASK) );
	fprintf( navdata_file, ";%s", str );
	//fprintf( navdata_file, "New data: \n");
	fprintf( navdata_file, ";nd_seq %u", (unsigned int) pnd->nd_seq);

	fprintf( navdata_file, "; raw_accs %04u; %04u; %04u; raw_gyros %04d; %04d;%04d ",
			(unsigned int) pnd->navdata_raw_measures.raw_accs[ACC_X],
			(unsigned int) pnd->navdata_raw_measures.raw_accs[ACC_Y],
			(unsigned int) pnd->navdata_raw_measures.raw_accs[ACC_Z],
			(int) pnd->navdata_raw_measures.raw_gyros[GYRO_X],
			(int) pnd->navdata_raw_measures.raw_gyros[GYRO_Y],
			(int) pnd->navdata_raw_measures.raw_gyros[GYRO_Z]

	        );

	fprintf( navdata_file, ";phys_accs % 5f; % 5f; % 5f;phys_gyros  % 6f; % 6f; % 6f; ",
			pnd->navdata_phys_measures.phys_accs[ACC_X],
			pnd->navdata_phys_measures.phys_accs[ACC_Y],
			pnd->navdata_phys_measures.phys_accs[ACC_Z],
			pnd->navdata_phys_measures.phys_gyros[GYRO_X],
			pnd->navdata_phys_measures.phys_gyros[GYRO_Y],
			pnd->navdata_phys_measures.phys_gyros[GYRO_Z]
        	);

	/*fprintf( navdata_file, ";gyros_offsets  % f; % f; % f",
			pnd->navdata_gyros_offsets.offset_g[GYRO_X],
			pnd->navdata_gyros_offsets.offset_g[GYRO_Y],
			pnd->navdata_gyros_offsets.offset_g[GYRO_Z] );*/


	/* Store information regarding the live video stream and the associated rate control */
	fprintf( navdata_file, "; frame_size %d;video_stream.frame_number %d; ",

			pnd->navdata_video_stream.frame_size,
			pnd->navdata_video_stream.frame_number

	);

	/* Store information regarding the HD storage stream */
	fprintf( navdata_file, " hdvideo_stream.frame_number: %d ",
			pnd->navdata_hdvideo_stream.frame_number
			);



	locked_ptr  = (int32_t*) &pnd->navdata_trackers_send.locked[0];
	point_ptr   = (screen_point_t*) &pnd->navdata_trackers_send.point[0];



	//fprintf( navdata_file, ";num_picture_decoded %u", (unsigned int) num_picture_decoded );

	sprintf( str, "%d.%06d", (int)time.tv_sec, (int)time.tv_usec);
	fprintf( navdata_file, "; %s \n", str );



    if((navdata_file_data != NULL) && (navdata_file_data->print != NULL))
  	  navdata_file_data->print(navdata_file);

	return C_OK;
}

C_RESULT ardrone_navdata_file_release( void )
{
  if( navdata_file != NULL )
  {
    navdata_file = NULL;

    fprintf(navdata_file_private,"\n");

    fclose( navdata_file_private );

    navdata_file_private = NULL;

    navdata_file_data = NULL;
  }

  return C_OK;
}
