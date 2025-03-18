
## \[ \overset\Rightarrow{\mu} = NI\overset\Rightarrow{A} \]
windings = 100;
radius_inductor = 30*10^(-3); ## meter
area_inductor = pi * radius_inductor ^2; ## meter^2
voltage = 8; ## volt
resistor_series = 320 * 10^0; ## ohm

## \[R_\text{L} = \rho \frac{l}{A} \]
resistivity_copper = 1.77 * 10^(-8); ## Ohm-meter
crosssectional_areal =(( 15*10^(-3) )/2)^2 * pi; ## meter^2
## \[    l = N \uptau r \]
length_inductor_wire = windings * 2 * pi * radius_inductor ## meter
resistor_inductor = resistivity_copper * length_inductor_wire / crosssectional_areal  ## ohm

## \[ i_\text{L}(\infty)\ = \frac{V}{R_\text{serie}+R_\text{L}} \]
resistance = resistor_series + resistor_inductor
current = voltage / resistance ## ampere


dipole = windings * current * area_inductor ## ampere * meter ^2


# Inductor inductance
## \[L = \mu_0\mu_r \frac{N^2 |\bivec{A}|}{l_\text{inductor}}  \]
magnetic_permeability = 4 * pi * 10^(-7); ## weber / ampere-meter
length_inductor = 0.5 * 10^(-2); # meter
inductance = magnetic_permeability * windings^2 * area_inductor / length_inductor ## Henry

# charge time
## \[    i_\text{L}(t) = i_\text{L}(\infty) - [i_\text{L}(\infty)-i_\text{L}(0+)]\eu^{-\frac{t}{\tau}}\]
time_constant = inductance / resistance ## second


f = figure;
set(f, "visible", "off")

t=0:(10^-6):(300*10^-6);
f = current - ((current - 0).* exp(-t ./time_constant));
plot(t, f), grid

print("MyPNG.png", "-dpng");

fclose all
fid = fopen("./filename.csv",'w');
A = [t; f]';
fprintf(fid,'%s, %s\n','time','current');
fprintf(fid,'%f, %f\n',A);
fclose(fid);
