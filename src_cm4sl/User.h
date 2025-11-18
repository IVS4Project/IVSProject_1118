/*
 *****************************************************************************
 *  CarMaker - Version 14.0.1
 *  Virtual Test Driving Tool
 *
 *  Copyright ©1998-2025 IPG Automotive GmbH. All rights reserved.
 *  www.ipg-automotive.com
 *****************************************************************************
 */

#ifndef _USER_H__
#define _USER_H__

#include <Global.h>
#include <Vehicle/MBSUtils.h>

#ifdef __cplusplus
extern "C" {
#endif

extern int UserCalcCalledByAppTestRunCalc;

#define N_USEROUTPUT 10

/* Struct for user variables. */
typedef struct tUser {
    /* For debugging purposes */
    double Out[N_USEROUTPUT];
} tUser;

extern tUser User;

int    User_Init_First(void);
int    User_Init(void);
void   User_PrintUsage(char const *Pgm);
char **User_ScanCmdLine(int argc, char **argv);
int    User_Start(void);
int    User_Register(void);
void   User_DeclQuants(void);
int    User_ShutDown(int ShutDownForced);
int    User_End(void);
void   User_Cleanup(void);

int User_TestRun_Start_atBegin(void);
int User_TestRun_Start_atEnd(void);
int User_TestRun_Start_StaticCond_Calc(void);
int User_TestRun_Start_Finalize(void);
int User_TestRun_RampUp(double dt);
int User_DrivMan_Calc(double dt);
int User_VehicleControl_Calc(double dt);
int User_Brake_Calc(double dt);
int User_Traffic_Calc(double dt);
int User_Calc(double dt);
int User_Check_IsIdle(int IsIdle);
int User_TestRun_End_First(void);
int User_TestRun_End(void);

void User_In(unsigned const CycleNo);
void User_Out(unsigned const CycleNo);

/* User_<> functions,
 * - called from SimCore and in CM_Main.c,
 * - already defined in SimCore.h
 */
int  User_Param_Get(void);
int  User_Param_Add(void);
int  User_ApoMsg_Eval(int channel, char *msg, int len, int who);
void User_ApoMsg_Send(double T, unsigned const CycleNo);

#define User_TestRun_Start User_TestRun_Start__deprecated_function__Change_to__User_TestRun_Start_XYZ;

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _USER_H__ */
