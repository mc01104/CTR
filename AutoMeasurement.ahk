#NoEnv  ; Recommended for performance and compatibility with future AutoHotkey releases.
#Warn  ; Enable warnings to assist with detecting common errors.
SendMode Input  ; Recommended for new scripts due to its superior speed and reliability.
SetWorkingDir %A_ScriptDir%  ; Ensures a consistent starting directory.


Escape::
ExitApp
Return

^j::
ctrGUI = Untitled - CTR
david = DAVID-LASERSCANNER v3.10.4
loopNum=1 
winactivate, %ctrGUI%
WinWaitActive, %ctrGUI%
MouseClick, left, 210, 230 
MouseClick, left, 210, 230 
Sleep, 500
MouseClick, right, 210, 230
Send, {Down}{Down}{Down}{Enter}
 
loopNum :=clipboard

Loop %loopNum%
{

	;=========== Move Configuration and Copy Configuration from ctr GUI =========
	winactivate, %ctrGUI%
	WinWaitActive, %ctrGUI%

	MouseClick, left, 310, 230 ; Move button
	Sleep, 30000

	MouseClick, left, 300, 300
	MouseClick, left, 300, 300 
	Sleep, 500
	MouseClick, right, 300, 300
	Send, {Down}{Down}{Down}{Enter}


	;=========== Scan and Save file as obj =========
	winactivate, %david%
	WinWaitActive, %david%

	MouseClick, left, 180, 430 ; Start button
	Sleep, 60000

	MouseClick, left, 240, 670 ; Save button
	Sleep, 1000

	Send, file^v.stl{Enter}
	Sleep, 1000
	Send, {Enter}
	;MouseClick, left, 960, 600
	Sleep, 1000
	;winactivate, %Save%
	;Send, {Enter}
	;MouseClick, left, 960, 580 ; Save button
	;Sleep, 10000


	
}
Return


