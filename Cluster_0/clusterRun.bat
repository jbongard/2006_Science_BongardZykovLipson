REM CCS account = jb382 
REM CCS type = batch 
REM CCS nodes = 5 
REM CCS minutes = 1380 
REM CCS requirements = 5@v3 
 
cd /D T:\ 
rmdir /S /Q t:\cluster_0 
REM make a subdirectory 
mkdir t:\cluster_0 
mkdir cluster_0\Data 
mkdir cluster_0\Files 
mkdir cluster_0\EE 
mkdir cluster_0\EE\Release 
mkdir cluster_0\Target 
mkdir cluster_0\Target\Release 
copy h:\users\jb382\cluster_0\EE\Release\*.*     t:\cluster_0\EE\Release 
copy h:\users\jb382\cluster_0\Target\Release\*.* t:\cluster_0\Target\Release 
cd cluster_0\EE\Release 
start /b Run_GA.bat 
cd ..\..\Target\Release 
start /b Run_Eval.bat 
cd ..\..\data 
:waitsomemore 
call sleep.bat 60 
if not exist est_0_2_end.dat goto waitsomemore 
copy *.* h:\users\jb382\cluster_0\data 
cd /D T:\ 
rmdir /S /Q t:\jb382 
ccrelease 
