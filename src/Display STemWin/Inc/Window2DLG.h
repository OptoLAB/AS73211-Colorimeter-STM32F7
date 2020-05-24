/*
 * Window2DLG.h
 *
 *  Created on: Apr 14, 2020
 *      Author: lazar
 */

#ifndef WINDOW2DLG_H_
#define WINDOW2DLG_H_

#include "DIALOG.h"

#define ID_WINDOW_20  (GUI_ID_USER + 0x23)
#define ID_BUTTON_20  (GUI_ID_USER + 0x24)
#define ID_TEXT_20  (GUI_ID_USER + 0x25)
#define ID_EDIT_20  (GUI_ID_USER + 0x26)
#define ID_TEXT_21  (GUI_ID_USER + 0x27)
#define ID_EDIT_21  (GUI_ID_USER + 0x28)
#define ID_EDIT_22  (GUI_ID_USER + 0x29)
#define ID_TEXT_22  (GUI_ID_USER + 0x2A)

WM_HWIN CreateWindow2(void);

#endif /* WINDOW2DLG_H_ */
