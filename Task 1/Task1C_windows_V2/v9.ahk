#Requires AutoHotkey v2.0

Numpad8:: ; Listen for 'Numpad8' key press
{
    Send("{Up}") ; Press 'up arrow'
    Sleep(400) ; Wait for 0.4 second
    Send("{Left}") ; Press 'left arrow'
    Sleep(1100) ; Wait for 0.6 second
    Send("{Up}") ; Press 'up arrow'
    Sleep(50) ; Wait for 0.4 second
    Send("q") ; Press 'q'
    Sleep(2000) ; Wait for 0.5 second
    Send("{Up}") ; Press 'up arrow'
    Sleep(2500) ; Wait for 2 seconds
    Send("{Left}") ; Press 'left arrow'
    Sleep(1100) ; Wait for 0.6 second
    Send("q") ; Press 'q'
}
