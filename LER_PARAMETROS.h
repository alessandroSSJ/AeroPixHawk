#ifndef LER_PARAMETROS
#define LER_PARAMETROS
#endif

/**************************************************************************************************************************
*$MCD Módulo de definição
*	  Nome : 	                Modulo de Leitura dos Parametros fundamentais de Voo
*	  Proprietário :         	Equipe AeroRio
*	  Projeto :		            SAE AeroDesign Brasil 2014
*	  Gestor :	 	            Alessandro Soares da Silva Junior
* 	  Arquivo : 	            LER_PARAMETROS.H
*	  Letras Identificadoras : 	LER
*	  Autor : 	                Alessandro Soares da Silva Junior
*
*$ED Descrição do módulo
*	O presente módulo tem como objetivo assinar todas as threads do PX4, ler e disponibilizar todos os paramêtros
*   fundamentais que o PX4 é capaz de disponibilizar.
*
*   O presente módulo necessita dos seguintes modulos a serem executados no cartão. (Script em bash)
*
*   	uorb start
*   	px4io start
*   	mpu6000 start
*       hmc5883 start
*       ms5611 start
*       adc start
*       sensors start
*       gps start
*       attitude_estimator_ekf start
*       position_estimator_inav start
*
*       PARA LIBERAR MAVLINK : mavlink start -d /dev/ttyS1 -b 57600 (Depois do -d a UART e depois do -b o baudrate)
*
***************************************************************************************************************************/

#ifdef LER_PARAMETROS_OWN
	#define LER_PARAMETROS_EXT
#else
	#define LER_PARAMETROS_EXT extern
#endif

/***** Declarações exportadas pelo módulo *****/

/* Tipo referência para os parametros */

typedef struct LER_parametros * LER_tpParametros ;


/***********************************************************************
*
*  $TC Tipo de dados: LER Condições de retorno
*
*
*  $ED Descrição do tipo
*     Condições de retorno das funções de leitura dos parâmetros
*
***********************************************************************/

   typedef enum {

         LER_CondRetOK           ,
              /* Leu corretamente                          */
         LER_CondRetError  ,
         	  /* Não Leu corretamente (Genérico)           */
         LER_CondRetAcelError    ,
              /* Não leu o acelerômetro corretamente       */
         LER_CondRetAttError     ,
              /* Não leu o mag corretamente                */
         LER_CondRetAltitudeError ,
         	  /* Não leu a altitude corretamente           */

} LER_tpCondRet ;

/***********************************************************************
*
*  $FC Função: LER  &Cria estrutura de parametros
*
*  $ED Descrição da função
*     Criar e inicializar uma estrutura de parâmetros
*
*  $FV Valor retornado
*     Se executou corretamente retorna a estrutura
*
*     Se ocorreu algum erro, retornará NULL.
*
***********************************************************************/

LER_tpParametros LER_CriarParam(void);

/***********************************************************************
*
*  $FC Função: LER  &Inicializar Módulo LER
*
*  $ED Descrição da função
*     Inicializa todas as variáveis globais necessárias ao bom funcionamento
*     do módulo. (OBS: Esta função deve ser chamada uma única vez)
*
*  $FV Valor retornado
*     Se executou corretamente retorna LER_CondRetOK.
*
*     Se ocorreu algum erro, retornará LER_CondRetError.
*
***********************************************************************/

LER_tpCondRet LER_Iniciar(void);

/***********************************************************************
*
*  $FC Função: LER  &Preencher Parametros
*
*  $ED Descrição da função
*     Preenche uma estrutura de parâmetros com todos os seus valores
*     ja filtrados e devidamente compensados.
*
*  $EP Parâmetros
*    pStructParam  - Ponteiro para uma estrutura de paramêtros
*
*  $FV Valor retornado
*     Se executou corretamente retorna LER_CondRetOK.
*
*     Se ocorreu algum erro, retornará LER_CondRetError.
*
***********************************************************************/

LER_tpCondRet LER_FillParam(LER_tpParametros pStructParam);

/***********************************************************************
*
*  $FC Função: LER  &Ler Pressão
*
*  $ED Descrição da função
*     Le e testa a pressão (em mbar) da estrutura de parametros
*
*  $EP Parâmetros
*    pStructParam  - Ponteiro para uma estrutura de paramêtros
*
*
***********************************************************************/

float LER_Pressao( LER_tpParametros pStructParam ) ;

/***********************************************************************
*
*  $FC Função: LER  &Ler Velocidade de Pitch
*
*  $ED Descrição da função
*     Le e testa a Velocidade de Pitch (em º/s) da estrutura de parametros
*
*  $EP Parâmetros
*    pStructParam  - Ponteiro para uma estrutura de paramêtros
*
*
***********************************************************************/

float LER_PitchSpeed( LER_tpParametros pStructParam ) ;

/***********************************************************************
*
*  $FC Função: LER  &Ler Velocidade de Roll
*
*  $ED Descrição da função
*     Le e testa a Velocidade de Roll (em º/s) da estrutura de parametros
*
*  $EP Parâmetros
*    pStructParam  - Ponteiro para uma estrutura de paramêtros
*
*
***********************************************************************/

float LER_RollSpeed( LER_tpParametros pStructParam ) ;

/***********************************************************************
*
*  $FC Função: LER  &Ler Velocidade de yaw
*
*  $ED Descrição da função
*     Le e testa a Velocidade de yaw da estrutura de parametros
*
*  $EP Parâmetros
*    pStructParam  - Ponteiro para uma estrutura de paramêtros
*
*
***********************************************************************/

float LER_YawSpeed( LER_tpParametros pStructParam ) ;

/***********************************************************************
*
*  $FC Função: LER  &Ler Angulo de Pitch
*
*  $ED Descrição da função
*     Le e testa o Angulo de Pitch da estrutura de parametros
*
*  $EP Parâmetros
*    pStructParam  - Ponteiro para uma estrutura de paramêtros
*
*
***********************************************************************/

float LER_PitchAngle( LER_tpParametros pStructParam ) ;

/***********************************************************************
*
*  $FC Função: LER  &Ler Angulo de Roll
*
*  $ED Descrição da função
*     Le e testa o Angulo de Roll da estrutura de parametros
*
*  $EP Parâmetros
*    pStructParam  - Ponteiro para uma estrutura de paramêtros
*
*
***********************************************************************/

float LER_RollAngle( LER_tpParametros pStructParam ) ;

/***********************************************************************
*
*  $FC Função: LER  &Ler Angulo de Yaw
*
*  $ED Descrição da função
*     Le e testa o Angulo de Yaw da estrutura de parametros
*
*  $EP Parâmetros
*    pStructParam  - Ponteiro para uma estrutura de paramêtros
*
*
***********************************************************************/

float LER_YawAngle( LER_tpParametros pStructParam ) ;

/***********************************************************************
*
*  $FC Função: LER  &Ler Aceleração no eixo x
*
*  $ED Descrição da função
*     Le e testa a aceleração em x da estrutura de parametros
*
*  $EP Parâmetros
*    pStructParam  - Ponteiro para uma estrutura de paramêtros
*
*
***********************************************************************/

float LER_AcelX( LER_tpParametros pStructParam ) ;

/***********************************************************************
*
*  $FC Função: LER  &Ler Aceleração no eixo y
*
*  $ED Descrição da função
*     Le e testa a aceleração em y da estrutura de parametros
*
*  $EP Parâmetros
*    pStructParam  - Ponteiro para uma estrutura de paramêtros
*
*
***********************************************************************/

float LER_AcelY( LER_tpParametros pStructParam ) ;

/***********************************************************************
*
*  $FC Função: LER  &Ler Aceleração no eixo z
*
*  $ED Descrição da função
*     Le e testa a aceleração em z da estrutura de parametros
*
*  $EP Parâmetros
*    pStructParam  - Ponteiro para uma estrutura de paramêtros
*
*
***********************************************************************/

float LER_AcelZ( LER_tpParametros pStructParam ) ;

/***********************************************************************
*
*  $FC Função: LER  &Ler Altitude
*
*  $ED Descrição da função
*     Le e testa a altitude da estrutura de parametros
*
*  $EP Parâmetros
*    pStructParam  - Ponteiro para uma estrutura de paramêtros
*
*
***********************************************************************/

float LER_Altitude( LER_tpParametros pStructParam ) ;




#undef LER_PARAMETROS
