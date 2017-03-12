#ifndef MY_CALCULATOR_BASE_H_
#define MY_CALCULATOR_BASE_H_
namespace my_calculator_base
{
	class calc_functions
	{
	public:
		// main methods implemented inside the calc_functions
		virtual void get_numbers(double number1, double number2) =0; // retrieve two numbers as input
		virtual double operation() =0; // define the mathematical operation
		//virtual ~calc_functions(){}
	protected:
		//calc_functions(){}
	};
};
#endif
