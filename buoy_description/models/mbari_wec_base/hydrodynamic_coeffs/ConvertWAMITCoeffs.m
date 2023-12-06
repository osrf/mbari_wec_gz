% This script converts WAMIT hydrodynamic data stored in .h5 format to the
% older .1 and .3 file formats described here:  https://www.wamit.com/manualv7.4/wamit_v74manualch13.html#x18-10700013
% This conversion script is not general purpose, it is coded
% specifically for the mbari_snl.h5 WAMIT results, and would need quite a
% bit of further work to be a general tool.
%
% Note also:  The radiation IRF's in the .h5 file must be integrated in
% time to match the form expected by the Gazebo WaveBodyInteraction plugin,
% this is done on lines 49-58 below.  This is a result of a formulation
% choice that was made differently in wecSIM than in the Gazebo simulator.

close all
clear all

filenm_base = 'mbari_snl'

WaveAngleDeg = h5read([filenm_base '.h5'],'/simulation_parameters/wave_dir');

%Wave Exciting IRF's
data_1 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/impulse_response_fun/components/f/1_1')';
data_2 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/impulse_response_fun/components/f/2_1')';
data_3 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/impulse_response_fun/components/f/3_1')';
data_4 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/impulse_response_fun/components/f/4_1')';
data_5 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/impulse_response_fun/components/f/5_1')';
data_6 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/impulse_response_fun/components/f/6_1')';


fid = fopen([filenm_base '_JR.3'],"w");
for i = 1:length(data_1);
  fprintf(fid,"%.4f  %.5f  %.5f  %.5f  %.5f  %.5f  %.5f\n",data_1(i,1),data_1(i,2),data_2(i,2),data_3(i,2),data_4(i,2),data_5(i,2),data_6(i,2));
end
fclose(fid);
clear data_1 data_2 data_3 data_4 data_5 data_6
clear i fid;


%Radiations IRF's
K_1_1 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/radiation_damping/impulse_response_fun/components/K/1_1')';
K_1_5 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/radiation_damping/impulse_response_fun/components/K/1_5')';
K_2_2 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/radiation_damping/impulse_response_fun/components/K/2_2')';
K_2_4 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/radiation_damping/impulse_response_fun/components/K/2_4')';
K_3_3 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/radiation_damping/impulse_response_fun/components/K/3_3')';
K_4_2 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/radiation_damping/impulse_response_fun/components/K/4_2')';
K_4_4 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/radiation_damping/impulse_response_fun/components/K/4_4')';
K_5_1 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/radiation_damping/impulse_response_fun/components/K/5_1')';
K_5_5 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/radiation_damping/impulse_response_fun/components/K/5_5')';
K_6_6 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/radiation_damping/impulse_response_fun/components/K/6_6')';

L_1_1 = cumtrapz(K_1_1(:,1),K_1_1(:,2));
L_1_5 = cumtrapz(K_1_5(:,1),K_1_5(:,2));
L_2_2 = cumtrapz(K_2_2(:,1),K_2_2(:,2));
L_2_4 = cumtrapz(K_2_4(:,1),K_2_4(:,2));
L_3_3 = cumtrapz(K_3_3(:,1),K_3_3(:,2));
L_4_2 = cumtrapz(K_4_2(:,1),K_4_2(:,2));
L_4_4 = cumtrapz(K_4_4(:,1),K_4_4(:,2));
L_5_1 = cumtrapz(K_5_1(:,1),K_5_1(:,2));
L_5_5 = cumtrapz(K_5_5(:,1),K_5_5(:,2));
L_6_6 = cumtrapz(K_6_6(:,1),K_6_6(:,2));

fid = fopen([filenm_base '_IR.1'],"w");
for i = 1:length(K_1_1);
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",K_1_1(i,1),1,1,L_1_1(i),L_1_1(i));
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",K_1_1(i,1),1,5,L_1_5(i),L_1_5(i));
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",K_1_1(i,1),2,2,L_2_2(i),L_2_2(i));
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",K_1_1(i,1),2,4,L_2_4(i),L_2_4(i));
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",K_1_1(i,1),3,3,L_3_3(i),L_3_3(i));
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",K_1_1(i,1),4,2,L_4_2(i),L_4_2(i));
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",K_1_1(i,1),4,4,L_4_4(i),L_4_4(i));
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",K_1_1(i,1),5,1,L_5_1(i),L_5_1(i));
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",K_1_1(i,1),5,5,L_5_5(i),L_5_5(i));
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",K_1_1(i,1),6,6,L_6_6(i),L_6_6(i));
end
fclose(fid);
clear K_1_1 K_1_5 K_2_2 K_2_4 K_3_3 K_4_2 K_4_4 K_5_1K K_5_5 K_6_6;
clear L_1_1 L_1_5 L_2_2 L_2_4 L_3_3 L_4_2 L_4_4 L_5_1 L_5_5 L_6_6;
clear i fid;

%Added Mass and Damping (Frequency Domain Coefficents)
AM_Inf = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/added_mass/inf_freq');

AM_1_1 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/added_mass/components/1_1')';
AM_1_5 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/added_mass/components/1_5')';
AM_2_2 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/added_mass/components/2_2')';
AM_2_4 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/added_mass/components/2_4')';
AM_3_3 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/added_mass/components/3_3')';
AM_4_2 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/added_mass/components/4_2')';
AM_4_4 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/added_mass/components/4_4')';
AM_5_1 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/added_mass/components/5_1')';
AM_5_5 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/added_mass/components/5_5')';
AM_6_6 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/added_mass/components/6_6')';

DMP_1_1 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/radiation_damping/components/1_1')';
DMP_1_5 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/radiation_damping/components/1_5')';
DMP_2_2 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/radiation_damping/components/2_2')';
DMP_2_4 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/radiation_damping/components/2_4')';
DMP_3_3 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/radiation_damping/components/3_3')';
DMP_4_2 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/radiation_damping/components/4_2')';
DMP_4_4 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/radiation_damping/components/4_4')';
DMP_5_1 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/radiation_damping/components/5_1')';
DMP_5_5 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/radiation_damping/components/5_5')';
DMP_6_6 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/radiation_damping/components/6_6')';

fid = fopen([filenm_base '.1'],"w");
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",-1.0,1,1,AM_Inf(1,1),0.0);
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",-1.0,1,5,AM_Inf(1,5),0.0);
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",-1.0,2,2,AM_Inf(2,2),0.0);
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",-1.0,2,4,AM_Inf(2,1),0.0);
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",-1.0,3,3,AM_Inf(3,3),0.0);
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",-1.0,4,2,AM_Inf(4,2),0.0);
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",-1.0,4,4,AM_Inf(4,4),0.0);
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",-1.0,5,1,AM_Inf(5,1),0.0);
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",-1.0,5,5,AM_Inf(5,5),0.0);
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",-1.0,6,6,AM_Inf(6,6),0.0);

for i = 1:length(AM_1_1(:,1));
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",AM_1_1(i,1),1,1,AM_1_1(i,2),DMP_1_1(i,2));
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",AM_1_1(i,1),1,5,AM_1_5(i,2),DMP_1_5(i,2));
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",AM_1_1(i,1),2,2,AM_2_2(i,2),DMP_2_2(i,2));
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",AM_1_1(i,1),2,4,AM_2_4(i,2),DMP_2_4(i,2));
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",AM_1_1(i,1),3,3,AM_3_3(i,2),DMP_3_3(i,2));
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",AM_1_1(i,1),4,2,AM_4_2(i,2),DMP_4_2(i,2));
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",AM_1_1(i,1),4,4,AM_4_4(i,2),DMP_4_4(i,2));
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",AM_1_1(i,1),5,1,AM_5_1(i,2),DMP_5_1(i,2));
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",AM_1_1(i,1),5,5,AM_5_5(i,2),DMP_5_5(i,2));
fprintf(fid,"%.4f  %d  %d  %.5f  %.5f\n",AM_1_1(i,1),6,6,AM_6_6(i,2),DMP_6_6(i,2));
end
fclose(fid);
clear  AM_1_1 AM_1_5   AM_2_2  AM_2_4  AM_3_3  AM_4_2  AM_4_4  AM_5_1  AM_5_5  AM_6_6;
clear DMP_1_1 DMP_1_5 DMP_2_2 DMP_2_4 DMP_3_3 DMP_4_2 DMP_4_4 DMP_5_1 DMP_5_5 DMP_6_6;
clear AM_Inf i fid;



%Wave Exciting Forces (Frequency Domain Coefficents)
X_re_1 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/re/1_1')';
X_re_2 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/re/2_1')';
X_re_3 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/re/3_1')';
X_re_4 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/re/4_1')';
X_re_5 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/re/5_1')';
X_re_6 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/re/6_1')';

X_im_1 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/im/1_1')';
X_im_2 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/im/2_1')';
X_im_3 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/im/3_1')';
X_im_4 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/im/4_1')';
X_im_5 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/im/5_1')';
X_im_6 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/im/6_1')';

X_mag_1 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/mag/1_1')';
X_mag_2 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/mag/2_1')';
X_mag_3 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/mag/3_1')';
X_mag_4 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/mag/4_1')';
X_mag_5 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/mag/5_1')';
X_mag_6 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/mag/6_1')';

X_ph_1 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/phase/1_1')';
X_ph_2 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/phase/2_1')';
X_ph_3 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/phase/3_1')';
X_ph_4 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/phase/4_1')';
X_ph_5 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/phase/5_1')';
X_ph_6 = h5read([filenm_base '.h5'],'/body1/hydro_coeffs/excitation/components/phase/6_1')';


fid = fopen([filenm_base '.3'],"w");
for i = 1:length(X_re_1);
fprintf(fid,"%.4f  %.4f  %d  %.5f  %.5f  %.5f  %.5f\n",X_re_1(i,1),WaveAngleDeg,1,X_mag_1(i,2),X_ph_1(i,2),X_re_1(i,2),X_im_1(i,2));
fprintf(fid,"%.4f  %.4f  %d  %.5f  %.5f  %.5f  %.5f\n",X_re_1(i,1),WaveAngleDeg,2,X_mag_2(i,2),X_ph_2(i,2),X_re_2(i,2),X_im_2(i,2));
fprintf(fid,"%.4f  %.4f  %d  %.5f  %.5f  %.5f  %.5f\n",X_re_1(i,1),WaveAngleDeg,3,X_mag_3(i,2),X_ph_3(i,2),X_re_3(i,2),X_im_3(i,2));
fprintf(fid,"%.4f  %.4f  %d  %.5f  %.5f  %.5f  %.5f\n",X_re_1(i,1),WaveAngleDeg,4,X_mag_4(i,2),X_ph_4(i,2),X_re_4(i,2),X_im_4(i,2));
fprintf(fid,"%.4f  %.4f  %d  %.5f  %.5f  %.5f  %.5f\n",X_re_1(i,1),WaveAngleDeg,5,X_mag_5(i,2),X_ph_5(i,2),X_re_5(i,2),X_im_5(i,2));
fprintf(fid,"%.4f  %.4f  %d  %.5f  %.5f  %.5f  %.5f\n",X_re_1(i,1),WaveAngleDeg,6,X_mag_6(i,2),X_ph_6(i,2),X_re_6(i,2),X_im_6(i,2));


end
