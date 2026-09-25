/**
  * @file    cli.h
  * @brief   Nhan lenh tu PC qua UART de dat duty, dat toc do va chinh he so PID
  *
  * Cac lenh (moi lenh mot dong, ket thuc bang xuong dong):
  *   s <rpm>     dat toc do, chay vong kin. Vi du: s 150 / s -120
  *   d <duty>    dat duty truc tiep, vong ho, PID nghi. Vi du: d 40 / d -60
  *   kp <v>      doi he so ti le.    Vi du: kp 0.12
  *   ki <v>      doi he so tich phan. Vi du: ki 0.5
  *   kd <v>      doi he so dao ham.  Vi du: kd 0.01
  *   demo        quay lai chay kich ban tu dong
  *   stop        tha troi dong co
  *   log 0|1     tat / bat dong log dinh ky
  *   ?           in trang thai hien tai
  */
#ifndef CLI_H
#define CLI_H

typedef enum
{
  CLI_MODE_DEMO = 0,  /* chay kich ban co san trong app.c */
  CLI_MODE_SPEED,     /* vong kin, toc do do nguoi dung dat */
  CLI_MODE_DUTY,      /* vong ho, duty do nguoi dung dat */
} CliMode_t;

void      Cli_Init(void);
void      Cli_Poll(void);        /* goi moi chu ky tu controlTask, khong block */
CliMode_t Cli_GetMode(void);
int       Cli_IsLogOn(void);

#endif /* CLI_H */
