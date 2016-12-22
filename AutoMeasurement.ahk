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
	Sleep, 20000

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
	Sleep, 1000
	;winactivate, %Save%
	;Send, {Enter}
	;MouseClick, left, 960, 580 ; Save button
	;Sleep, 10000

	;=========== Save file as stl =====================
	;winactivate, %david%
	;WinWaitActive, %david%

	;MouseClick, left, 1740, 160 ; + button in 'List of Scans'
	;Sleep, 1000
	;Send, file^v{Enter}
	;Sleep, 10000

	;MouseClick, left, 1660, 120 ; Check box in 'List of Scans'
	;Sleep, 500
	;MouseClick, left, 1850, 160 ; Save button in 'List of Scans'
	;Sleep, 500
	;Send, file^v.stl{Enter}
	;Sleep, 1000
	;Send, {Enter}	; Save as stl
	;Sleep, 10000
	;Send, {Enter}
	;MouseClick, left, 1775, 160 ; - button in 'List of Scans'
	;Sleep, 1000	

	
}
Return


