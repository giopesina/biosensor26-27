&adc {
	status = "okay";
};

&pinctrl {
	pwm20_default: pwm20_default {
		group1 {
			psels = <NRF_PSEL(PWM_OUT0, 1, 5)>;
		};
	};

	pwm20_sleep: pwm20_sleep {
		group1 {
			psels = <NRF_PSEL(PWM_OUT0, 1, 5)>;
			low-power-enable;
		};
	};
};

&pwm20 {
	status = "okay";
	pinctrl-0 = <&pwm20_default>;
	pinctrl-1 = <&pwm20_sleep>;
	pinctrl-names = "default", "sleep";
};