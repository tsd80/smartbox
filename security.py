def isPassword ():
	button_state1 = GPIO.input(i_sec_button_gpio_number)
	if button_state1 == False:
		time.sleep(1)
		button_state2 = GPIO.input(i_sec_button_gpio_number)
		if button_state2 == False:
			ret=True
		else:
			ret=False
	else:
		ret=False
	return ret
