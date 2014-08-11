/***************************************************************************
*  $MCI Módulo de implementação: LER Leitura dos Parâmetros fundamentais
*
*  Arquivo gerado:              LER_PARAMETROS.c
*  Letras identificadoras:      LER
*
*
*  Projeto: SAE AeroDesign Brasil 2014
*  Gestor:  Alessandro Soares da Silva Junior
*  Autores: Alessandro Soares da Silva Junior
*
*
***************************************************************************/

#ifndef _STDIO
#define _STDIO
#include <stdio.h>
#endif

#ifndef _STDLIB
#define _STDLIB
#include <stdlib.h>
#endif

#ifndef _UORB
#define _UORB
#include <uORB/uORB.h>
#endif

#ifndef _DRIVERS
#define _DRIVERS
#include <drivers/drv_hrt.h>
#endif

#ifndef _ATTT
#define _ATTT
#include <uORB/topics/vehicle_attitude.h>
#endif

#ifndef _SENSOR
#define _SENSOR
#include <uORB/topics/sensor_combined.h>
#endif

#ifndef _POLL
#define _POLL
#include <poll.h>
#endif

#ifndef _GLOBAL
#define _GLOBAL
#include <uORB/topics/vehicle_local_position.h>
#endif

#define LER_PARAMETROS_OWN
#include "LER_PARAMETROS.h"
#undef LER_PARAMETROS_OWN

#define ATT_FD    2
#define GLOBAL_FD 1
#define SENSOR_FD 0

/***********************************************************************
*
*  $TC Tipo de dados: LER - Parametros
*
*
***********************************************************************/

typedef struct LER_parametros {
	float pressao    ;                         /* Pressão em MILIBAR                            */
	float pitchSpeed ;                         /* Velocidade de Pitch em º/s                */
	float rollSpeed  ;                         /* Velocidade de Roll  em º/s                */
	float yawSpeed   ;                         /* Velocidade de yaw   em º/s                */
	float pitch      ;                         /* Angulo de yaw  em º                       */
	float roll       ;                         /* Angulo de roll em º                       */
	float yaw        ;                         /* Angulo de yaw  em º                       */
	float ax         ;                         /* Aceleração no eixo x em m/s²              */
	float ay         ;                         /* Aceleração no eixo y em m/s²              */
	float az         ;                         /* Aceleração no eixo z em m/s²              */
	float altura     ;                         /* Altura em relação ao home point em metros */

} LER_parametros;

/***** Constantes Globais *****/

static const float CONVERT_RAD_INTO_GRAU = 57.29747 ;

/***** Variáveis Globais ******/

static int att_fd           ;

static int global_fd        ;

static int sensor_fd        ;

static struct pollfd fds[3] ;


/***** Protótipos das funções encapuladas no módulo *****/

	static LER_tpCondRet aquisitarAceleracao    (float *Acel , float *pressao )  ;
	static LER_tpCondRet aquisitarAtitudes      (float *Att , float *attRates )  ;
	static LER_tpCondRet aquisitarAltitude      (float *altura   )               ;

/*****  Código das funções exportadas pelo módulo  *****/

/***************************************************************************
*
*  Função: LER  &Criar estrutura
*  ****/

	LER_tpParametros LER_CriarParam(void)
	{
		LER_parametros *pParam = NULL;

		pParam = (LER_parametros *) malloc(sizeof(LER_parametros)) ;

		return pParam ;
	}

/***************************************************************************
*
*  Função: LER  & Inicializar a estrutura do módulo
*  ****/

	LER_tpCondRet LER_Iniciar(void)
	{

		att_fd    = orb_subscribe( ORB_ID( vehicle_attitude ) )         ;

		global_fd = orb_subscribe( ORB_ID( vehicle_local_position ) )   ;

		sensor_fd = orb_subscribe( ORB_ID( sensor_combined ))           ;

		fds[ATT_FD].fd        = att_fd                                  ;
		fds[ATT_FD].events    = POLLIN                                  ;

		fds[GLOBAL_FD].fd     = global_fd                               ;
		fds[GLOBAL_FD].events = POLLIN                                  ;

		fds[SENSOR_FD].fd     = sensor_fd                               ;
		fds[SENSOR_FD].events = POLLIN                                  ;

		orb_set_interval( att_fd    , 100   )                           ;
		orb_set_interval( global_fd , 100   )                           ;
		orb_set_interval( sensor_fd , 100   )                           ;

		return LER_CondRetOK ;
	}

/***************************************************************************
*
*  Função: LER  & Preenche a estrutura com os parâmetros fundamentais
*  ****/

	LER_tpCondRet LER_FillParam(LER_tpParametros pStructParam)
	{
		float acel[3]         ;
		float att [3]         ;
		float attRates [3]    ;
		float pressao         ;
		float altitude        ;

		LER_tpCondRet retAcel    ; /* Retorno da saída do acelerometro             */
		LER_tpCondRet retAtt     ; /* Retorno da saída do magnetômetro             */
		LER_tpCondRet retAttRate ; /* Retorno da saída da derivada do magnetômetro */
		LER_tpCondRet retPressao ; /* Retorno da saída do barômetro                */
		LER_tpCondRet retHeight  ; /* Retorno da saída do calculo da altura        */

		/* Aquisitar os parâmetros e analisar resultados */

		/* Verifica se teve dados no último segundo */

		int poll_ret = poll( fds , 3 , 200 )               ;

		/* handling resultado */

		if ( poll_ret == 0 ) /* Sem data */
		{
			printf(" [Aero] Sem dados por um segundo\n ") ;
			return LER_CondRetError                       ;
		}

		else if ( poll_ret < 0) /* Erro bizarro */
		{
			printf(" [Aero] Erro bizarro\n ")             ;
			return LER_CondRetError                       ;
		}
		else  /* Teve parametros ! */
		{

			retAcel = aquisitarAceleracao( acel , &pressao )    ;

			if ( retAcel == LER_CondRetAcelError )
			{
				printf("Erro no sensor combined\n")             ;
			}

			retAtt = aquisitarAtitudes ( att , attRates )       ;

			if ( retAtt == LER_CondRetAttError )
			{
				printf("Erro no attitude\n")                    ;
			}

			retHeight = aquisitarAltitude ( &altitude )         ;

			if ( retHeight == LER_CondRetAltitudeError )
			{
				printf("Erro no global\n")                      ;
			}

		}

		/* Preencher os parâmetros caso os dados sejam válidos */

		if ( retAcel == LER_CondRetOK )
		{
			pStructParam->ax = acel[0]  ;
			pStructParam->ay = acel[1]  ;
			pStructParam->az = acel[2]  ;
		}

		if ( retAtt == LER_CondRetOK )
		{
			pStructParam->roll  = att[0] * CONVERT_RAD_INTO_GRAU ;
			pStructParam->pitch = att[1] * CONVERT_RAD_INTO_GRAU ;
			pStructParam->yaw   = att[2] * CONVERT_RAD_INTO_GRAU ;
		}

		if ( retAttRate == LER_CondRetOK )
		{
			pStructParam->rollSpeed  = attRates[0] * CONVERT_RAD_INTO_GRAU ;
			pStructParam->pitchSpeed = attRates[1] * CONVERT_RAD_INTO_GRAU ;
			pStructParam->yawSpeed   = attRates[2] * CONVERT_RAD_INTO_GRAU ;
		}

		if ( retPressao == LER_CondRetOK )
		{
			pStructParam->pressao = pressao ;
		}

		if ( retHeight == LER_CondRetOK )
		{
			pStructParam->altura  = altitude ;
		}

		/* Verifica se algumas das anteriores não funcionou direito */

		if( retAcel    != LER_CondRetOK ||
			retAtt     != LER_CondRetOK ||
			retAttRate != LER_CondRetOK ||
			retPressao != LER_CondRetOK ||
			retHeight  != LER_CondRetOK   )
		{
			return LER_CondRetError;
		}

		return LER_CondRetOK ;

	}

	/***************************************************************************
	*
	*  Função: LER  & Leitura propriamente dita da pressão
	*  ****/

	float LER_Pressao( LER_tpParametros pStructParam )
	{
		float pressao ;

		pressao = pStructParam->pressao ;

		if ( pressao < 0 )
			return -1 ;

		return pressao ;
	}

	/***************************************************************************
	*
	*  Função: LER  & Leitura propriamente dita do PitchSpeed
	*  ****/

	float LER_PitchSpeed( LER_tpParametros pStructParam )
	{
		return pStructParam->pitchSpeed ;
	}

	/***************************************************************************
	*
	*  Função: LER  & Leitura propriamente dita do RollSpeed
	*  ****/

	float LER_RollSpeed( LER_tpParametros pStructParam )
	{
		return pStructParam->rollSpeed ;
	}

	/***************************************************************************
	*
	*  Função: LER  & Leitura propriamente dita do YawSpeed
	*  ****/

	float LER_YawSpeed( LER_tpParametros pStructParam )
	{
		return pStructParam->yawSpeed ;
	}

	/***************************************************************************
	*
	*  Função: LER  & Leitura propriamente dita do PitchAngle
	*  ****/

	float LER_PitchAngle( LER_tpParametros pStructParam )
	{
		return pStructParam->pitch ;
	}

	/***************************************************************************
	*
	*  Função: LER  & Leitura propriamente dita do RollAngle
	*  ****/

	float LER_RollAngle( LER_tpParametros pStructParam )
	{
		return pStructParam->roll ;
	}

	/***************************************************************************
	*
	*  Função: LER  & Leitura propriamente dita do YawAngle
	*  ****/

	float LER_YawAngle( LER_tpParametros pStructParam )
	{
		return pStructParam->yaw ;
	}

	/***************************************************************************
	*
	*  Função: LER  & Leitura propriamente dita da aceleração em x
	*  ****/

	float LER_AcelX( LER_tpParametros pStructParam )
	{
		return pStructParam->ax ;
	}

	/***************************************************************************
	*
	*  Função: LER  & Leitura propriamente dita da aceleração em y
	*  ****/

	float LER_AcelY( LER_tpParametros pStructParam )
	{
		return pStructParam->ay ;
	}

	/***************************************************************************
	*
	*  Função: LER  & Leitura propriamente dita da aceleração em z
	*  ****/

	float LER_AcelZ( LER_tpParametros pStructParam )
	{
		return pStructParam->az ;
	}

	/***************************************************************************
	*
	*  Função: LER  & Leitura propriamente dita da aceleração em z
	*  ****/

	float LER_Altitude( LER_tpParametros pStructParam )
	{
		return pStructParam->altura ;
	}

	/***************************************************************************
	*
	*  Função: LER  & Aquisitar o parâmetro aceleração
	*  ****/

	static LER_tpCondRet aquisitarAceleracao( float *Acel , float *pressao )
	{

		struct sensor_combined_s raw ;

			if ( fds[SENSOR_FD].revents & POLLIN ) /* Checando se teve parametros novos e copiar se for o caso */
			{
				orb_copy( ORB_ID( sensor_combined ) , sensor_fd, &raw)                 ;

				Acel[0]  = raw.accelerometer_m_s2[0]                                   ;
				Acel[1]  = raw.accelerometer_m_s2[1]                                   ;
				Acel[2]  = raw.accelerometer_m_s2[2]                                   ;

				*pressao = raw.baro_pres_mbar                                          ;

				return LER_CondRetOK                                                   ;
			}

		return LER_CondRetAcelError                                                    ;

	}

	/***************************************************************************
	*
	*  Função: LER  & Aquisitar as atitudes
	*  ****/

	static LER_tpCondRet aquisitarAtitudes( float *Att , float *attRates )
	{

		struct vehicle_attitude_s raw ;

			if ( fds[ATT_FD].revents & POLLIN ) /* Checando se teve parametros novos e copiar se for o caso */
			{
				orb_copy( ORB_ID( vehicle_attitude ) , att_fd, &raw)                 ;

				Att[0]      = raw.roll                                               ;
				Att[1]      = raw.pitch                                              ;
				Att[2]      = raw.yaw                                                ;

				attRates[0] =raw.rollspeed                                           ;
				attRates[1] =raw.pitchspeed                                          ;
				attRates[2] =raw.yawspeed                                            ;


				return LER_CondRetOK                                                 ;
			}

		return LER_CondRetAttError ;
	}

	/***************************************************************************
	*
	*  Função: LER  & Aquisitar as atitudes rates
	*  ****/

	static LER_tpCondRet aquisitarAltitude( float *altura )
	{

		struct vehicle_local_position_s raw ;

		if ( fds[ATT_FD].revents & POLLIN ) /* Checando se teve parametros novos e copiar se for o caso */
		{
			orb_copy( ORB_ID( vehicle_local_position ) , global_fd, &raw)                 ;

			*altura = raw.z ;

			return LER_CondRetOK ;
		}

		return LER_CondRetAltitudeError ;
	}


