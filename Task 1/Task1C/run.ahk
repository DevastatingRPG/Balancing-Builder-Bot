#Requires AutoHotkey v2.0

Numpad8:: ; Listen for 'Numpad8' key press
{
    Send("{Up}") ; Press 'up arrow'
    Sleep(30) ; Wait for 0.4 second
    Send("{Left}") ; Press 'left arrow'
    Sleep(410) ; Wait for 0.6 second
    Send("q") ; Press 'q'
    Sleep(860) ; Wait for 0.5 second
    Send("q") ; Press 'q'
    Sleep(10) ; Wait for 0.4 second
    Send("{Up}") ; Press 'up arrow'
    Send("{Up}") ; Press 'up arrow'
}