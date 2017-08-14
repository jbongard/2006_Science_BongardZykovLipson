def ToAVI():

    import os;
    
    dirString = 'C:\\PostDoc\\Papers\\2006\\2006_Nature\\Code\\Target\\Release\\frame\\'
    commandStr = dirString + 'bmp2avi';
    os.system(commandStr);

def RenameBMPs(numFrames):

    import os;
    
    dirString = 'C:\\PostDoc\\Papers\\2006\\2006_Nature\\Code\\Target\\Release\\frame\\'

    for i in range(0,numFrames):
        if ( i < 10 ):
            prefixStr = '0000';
        elif (i < 100 ):
            prefixStr = '000';
        elif (i < 1000 ):
            prefixStr = '00';
        else:
            prefixStr = '0';

        commandStr = 'rename ' + dirString + str(i) + '.bmp ' + prefixStr + str(i) + '.bmp'
        print commandStr
        os.system(commandStr);
        
def ToBMPs(numFrames):

    import os;

    dirString = 'C:\\PostDoc\\Papers\\2006\\2006_Nature\\Code\\Target\\Release\\frame\\'
    
    for i in range(0,numFrames):
        commandStr = dirString + 'CONV ' + dirString + str(i) + '.ppm ' + dirString + str(i) + '.bmp';
        #print commandStr
        os.system(commandStr);
        commandStr = 'del ' + dirString + str(i) + '.ppm'
        os.system(commandStr);

    RenameBMPs(numFrames);

def ExtractBestGaits():

    dirString = 'C:\\PostDoc\\Papers\\2006\\2006_Nature\\Data\\Damage_Gaits\\'

    for i in range(0,26):
        fileName = dirString + 'exp_' + str(i) + '_2_brain0.dat';
        f = open(fileName,'r');

        for j in range(0,199*11):
            lineStr = f.readline();

        print i
        lineStr = f.readline();
        lineStr1 = f.readline();
        lineStr1 = lineStr1[0:len(lineStr1)-1];
        print lineStr1
        
        lineStr2 = f.readline();
        lineStr2 = lineStr2[0:len(lineStr2)-1];
        print lineStr2
        
        lineStr3 = f.readline();
        lineStr3 = lineStr3[0:len(lineStr3)-1];
        print lineStr3
        
        lineStr4 = f.readline();
        lineStr4 = lineStr4[0:len(lineStr4)-1];
        print lineStr4
        
        lineStr5 = f.readline();
        lineStr5 = lineStr5[0:len(lineStr5)-1];
        print lineStr5

        print lineStr2
        print lineStr3
        print lineStr4
        print lineStr5
        print lineStr1
        print '\n'
        
        lineStr6 = f.readline();
        lineStr7 = f.readline();
        lineStr8 = f.readline();
        lineStr9 = f.readline();
        lineStr10 = f.readline();
            
        f.close();
        
def CopyGaitFiles(runNum):

    import os;
    import shutil;

    directory = '..\\Files\\Sensors.dat';
    os.remove(directory);
    
    oldFileName = '..\\Gait_' + str(runNum) + '\\Data_t16_0_29\\est_0_2_body15.dat'
    newFileName = '..\\Files\\Body.dat'
    shutil.copyfile(oldFileName,newFileName);

    oldFileName = '..\\Gait_' + str(runNum) + '\\Data\\exp_' + str(runNum) + '_2_brain0_full.dat'
    newFileName = '..\\Files\\Brain.dat'
    shutil.copyfile(oldFileName,newFileName);
    
def DealWithBatch(i,j):

    fileName = '..\\Data_t16_0_29\\est_' + str(i);
    fileName = fileName + '_' + str(j) + '_body15.dat';

    f = open(fileName,'r');
    
    fileString = f.read();
    markerFound = fileString.find('-eval');
    print markerFound;
    
    f.close();

def ExtractBodies(startBody,endBody,numToSkip):

    fileName = '..\\Data_t16_0_29\\est_0_2_bodies.dat'
    f = open(fileName,'r');

    currIndex = 0;

    while ( currIndex < startBody ):
        
        for i in range(0,240):
            lineString = f.readline();
        currIndex = currIndex + 1;

    fileName = '..\\Files\\Body.dat'
    fOut = open(fileName,'w');

    fOut.write( str(endBody-startBody+1) + '\n' );
    
    while ( currIndex <= endBody ):

        for i in range(0,240):
            lineString = f.readline();
            fOut.write(lineString);

        for j in range(0,numToSkip-1):
            for i in range(0,240):
                lineString = f.readline();
            
        currIndex = currIndex + 1;

    fOut.close(); 
    f.close();

def ExtractBrains(brainIndex,copies):

    fileName = '..\\Data_t16_0_29\\exp_0_2_brain' + str(brainIndex) + '.dat';
    f = open(fileName,'r');

    brain = f.read();

    f.close();

    fileName = '..\\Files\\Brain.dat'
    fOut = open(fileName,'w');

    fOut.write( str(copies) + '\n' );

    for i in range(0,copies):
        fOut.write(brain);

    fOut.close();
    
def CountFracCorrect(startRun,endRun):

    t = [1, 1, 1];
    corrects = t,t,t,t,t,t,t,t,t,t,t,t,t,t,t,t,t,t,t,t,t,t,t,t,t,t,t,t,t;
    print corrects;
    
    for i in range(startRun,endRun+1):
        for j in range(0,2+1):

            fileName = '..\\Data_t16_0_29\\est_' + str(i);
            fileName = fileName + '_' + str(j) + '_body15.dat';
            f = open(fileName,'r');
            fileString = f.read();
            jointFound01 = fileString.find('-connect 0 1');
            jointFound02 = fileString.find('-connect 0 2');
            jointFound03 = fileString.find('-connect 0 3');
            jointFound04 = fileString.find('-connect 0 4');
            jointFound15 = fileString.find('-connect 1 5');
            jointFound26 = fileString.find('-connect 2 6');
            jointFound37 = fileString.find('-connect 3 7');
            jointFound48 = fileString.find('-connect 4 8');
            jointFound = ( (jointFound01>-1) &
                    (jointFound02>-1) &
                    (jointFound03>-1) &
                    (jointFound04>-1) &
                    (jointFound15>-1) &
                    (jointFound26>-1) &
                    (jointFound37>-1) &
                    (jointFound48>-1) );
            corrects[i-1][j] = jointFound;
            f.close();
            #print i, j;
        print corrects[i-1];
            
        
def CreateClusterScripts(endMachine):
    
    currMachine = 1;

    while currMachine <= endMachine:

        CreateDirectories(currMachine);
        CreateRunFile(currMachine);
        CreateClusterScript(currMachine);
        
        currMachine = currMachine + 1;
##            
##        if ( useEEA ):
##            CreateRunFile(currMachine,1,runNum,0,useEEA);
##            CreateRunFile(currMachine,2,runNum,1,useEEA);
##        else:
##            CreateRunFile(currMachine,1,runNum,2,useEEA);
##            CreateRunFile(currMachine,2,runNum,3,useEEA);
##        
##        runNum = runNum + 1;
##
##        currMachine = currMachine + 1;
##
    CreateMachineScript(0,endMachine);

def CreateDirectories(currMachine):

    import os;
    import shutil;

    dirName = '..\\Gait_' + str(currMachine);
    
    os.mkdir(dirName);

    newDir = dirName + '\\Data'
    os.mkdir(newDir);

    newDir = dirName + '\\Data_t16_0_29'
    os.mkdir(newDir);

    #oldFileName = '..\\Data_t16_0_29\\est_' + str(currMachine) + '_' + str(2) + '_body15.dat';
    #newFileName = '..\\Gait_' + str(currMachine) + '\\Data_t16_0_29\\est_' + str(currMachine) + '_' + str(2) + '_body15.dat';
    oldFileName = '..\\Data_t16_0_29\\est_0_' + str(2) + '_body15.dat';
    newFileName = '..\\Gait_' + str(currMachine) + '\\Data_t16_0_29\\est_0_' + str(2) + '_body15.dat';

    shutil.copyfile(oldFileName,newFileName);

    ChangeClusterScript(currMachine);
    
    newDir = dirName + '\\EE'
    os.mkdir(newDir);

    newDir = dirName + '\\EE\\Release'
    os.mkdir(newDir);

    oldFileName = '..\\Gait_0\\EE\\Release\\GA_Camera.exe';
    newFileName = '..\\Gait_' + str(currMachine) + '\\EE\\Release\\GA_Camera.exe';
    shutil.copyfile(oldFileName,newFileName);

    oldFileName = '..\\Gait_0\\EE\\Release\\Process.exe';
    newFileName = '..\\Gait_' + str(currMachine) + '\\EE\\Release\\Process.exe';
    shutil.copyfile(oldFileName,newFileName);
    
    newDir = dirName + '\\Files'
    os.mkdir(newDir);

    newDir = dirName + '\\Target'
    os.mkdir(newDir);

    newDir = dirName + '\\Target\\Release'
    os.mkdir(newDir);

    oldFileName = '..\\Gait_0\\Target\\Release\\MorphEngine.exe';
    newFileName = '..\\Gait_' + str(currMachine) + '\\Target\\Release\\MorphEngine.exe';
    shutil.copyfile(oldFileName,newFileName);

    oldFileName = '..\\Gait_0\\Target\\Release\\Process.exe';
    newFileName = '..\\Gait_' + str(currMachine) + '\\Target\\Release\\Process.exe';
    shutil.copyfile(oldFileName,newFileName);

    oldFileName = '..\\Gait_0\\Target\\Release\\Run_Eval.bat';
    newFileName = '..\\Gait_' + str(currMachine) + '\\Target\\Release\\Run_Eval.bat';
    shutil.copyfile(oldFileName,newFileName);

def CreateClusterScript(currMachine):

    fileName = '..\\Gait_' + str(currMachine);
    
    fileName = fileName + '\\clusterRun.bat';

    f = open(fileName,'w');

    f.write('REM CCS account = jb382 \n');
    f.write('REM CCS type = batch \n');
    f.write('REM CCS nodes = 2 \n');
    f.write('REM CCS minutes = 480 \n');
    f.write('REM CCS requirements = 2@v3 \n \n');

    f.write('cd /D T:\ \n');
    f.write('rmdir /S /Q t:\Gait_' + str(currMachine) + ' \n');
    f.write('REM make a subdirectory \n');
    f.write('mkdir t:\Gait_' + str(currMachine) + ' \n');
    f.write('mkdir Gait_' + str(currMachine) + '\Data \n');
    f.write('mkdir Gait_' + str(currMachine) + '\Data_t16_0_29 \n');
    f.write('mkdir Gait_' + str(currMachine) + '\Files \n');
    f.write('mkdir Gait_' + str(currMachine) + '\EE \n');
    f.write('mkdir Gait_' + str(currMachine) + '\EE\Release \n');
    f.write('mkdir Gait_' + str(currMachine) + '\Target \n');
    f.write('mkdir Gait_' + str(currMachine) + '\Target\Release \n');
    f.write('copy h:\users\jb382\Gait_' + str(currMachine) + '\Data_t16_0_29\*.*  t:\Gait_' + str(currMachine) + '\Data_t16_0_29 \n');
    f.write('copy h:\users\jb382\Gait_' + str(currMachine) + '\EE\Release\*.*     t:\Gait_' + str(currMachine) + '\EE\Release \n');
    f.write('copy h:\users\jb382\Gait_' + str(currMachine) + '\Target\Release\*.* t:\Gait_' + str(currMachine) + '\Target\Release \n');

    f.write('cd Gait_' + str(currMachine) + '\EE\Release \n');
    f.write('start /b Run_GA.bat \n');
    f.write('cd ..\..\Target\Release \n');
    f.write('start /b Run_Eval.bat \n');

    f.write('cd ..\..\data \n');

    f.write(':waitsomemore \n');
    f.write('call sleep.bat 60 \n');
    f.write('if not exist est_' + str(currMachine) + '_2_end.dat goto waitsomemore \n');

    f.write('copy *.* h:\users\jb382\Gait_' + str(currMachine) + '\data \n');

    f.write('cd /D T:\ \n');
    f.write('rmdir /S /Q t:\jb382 \n');
    f.write('ccrelease \n');

    f.close();

def CreateRunFile(currMachine):

    dirName = '..\\Gait_' + str(currMachine) + '\\EE\\Release\\';

    fileName = dirName + '\\Run_GA.bat';

    f = open(fileName,'w');

    strCommand = 'GA_Camera -r ';
    strCommand = strCommand + str(currMachine);

    strCommand = strCommand + ' -it';
    
    f.write(strCommand);
    
    f.close();

def CreateMachineScript(startMachine,currMachine):

    fileName = '..\\..\\startMachines.dat';

    f = open(fileName,'w');
    
    for i in range(startMachine,currMachine+1):
        strCommand = 'cd ..\\Gait_' + str(i) + '\n';
        strCommand = strCommand + 'ccsubmit clusterRun.bat\n\n';
        f.write(strCommand);

    f.close();
    
def ChangeRunLengths(startRun,endRun):

    for i in range(startRun,endRun+1):

        ChangeClusterScript(i);
        
        for j in range(1,2+1):
           ChangeRunLength(i,j)

def ChangeClusterScript(currMachine):
    
    #fileName = '..\\Gait_' + str(currMachine) + '\\Data_t16_0_29\\est_' + str(currMachine) + '_' + str(2) + '_body15.dat';
    fileName = '..\\Gait_' + str(currMachine) + '\\Data_t16_0_29\\est_0_' + str(2) + '_body15.dat';
    f = open(fileName,'r')
    script = f.read()
    f.close()

    oldString = '-evaluationPeriod 4000';
    newString = '-evaluationPeriod 3000';

    script = script.replace(oldString,newString)
    
    f=open(fileName,'w')
    f.write(script)
    f.close()
    
def ChangeRunLength(runNum,expRegime):

    fileName = '..\\Pareto_' + str(runNum)
    fileName = fileName + '\\run' + str(expRegime) + '.bat'

    f = open(fileName,'r')
    script = f.read()
    f.close()
            
    oldString = '-c 16'
    newString = '-c 8'
    script = script.replace(oldString,newString)

    oldString = '-null'
    newString = '-null -l'
    script = script.replace(oldString,newString)
    
    f=open(fileName,'w')
    f.write(script)
    f.close()
    
    print fileName
    
def RemoveNonNumerics(startRun,endRun):

    for i in range(startRun,endRun+1):
        for j in range(0,3+1):
            RemoveNons(i,j,'_real.dat')
            #RemoveNons(i,j,'_banked.dat')

def RemoveNons(runNum,expRegime,fileType):

    #fileName = '..\\Pareto_' + str(runNum)
    #fileName = fileName + '\\data\\run' + str(runNum)
    #fileName = '..\\Release\\backup\\run' + str(runNum)
    fileName = '..\\lac_Noise\\run' + str(runNum)
    fileName = fileName + '_' + str(expRegime) + fileType

    f = open(fileName,'r')
    script = f.read()
    f.close()
            
    oldString = '1.#INF'
    newString = '1e+10'
    script = script.replace(oldString,newString)

    f=open(fileName,'w')
    f.write(script)
    f.close()
    
    print fileName

def Ren(oldRunNum,newRunNum,experimentType,fileType):

    print 1
    
def Rename(oldRunNum,newRunNum):

    Ren(oldRunNum,newRunNum,0,'best')
    Ren(oldRunNum,newRunNum,0,'experiments')
    Ren(oldRunNum,newRunNum,0,'fit')

    Ren(oldRunNum,newRunNum,1,'best')
    Ren(oldRunNum,newRunNum,1,'experiments')
    Ren(oldRunNum,newRunNum,1,'fit')

    Ren(oldRunNum,newRunNum,2,'best')
    Ren(oldRunNum,newRunNum,2,'experiments')
    Ren(oldRunNum,newRunNum,2,'fit')

def RenameRuns(oldStart,oldEnd,newStart,newEnd):

    j = newStart
    for i in range(oldStart,oldEnd+1):

        Rename(i,j)
        print i
        print j
        j = j + 1
    
def GetDirName(runNum,useQuadruped):

    if useQuadruped:
        preamble = 'C:\\PostDoc\\Failure\\Traditional\\run'
    else:
        preamble = 'C:\\PostDoc\\Failure\\Traditional\\run'
        
    fileName = preamble + str(runNum) + '_failures.dat'
    return fileName

def GetDamageDirName(runNum):

    preamble = '..\\DE_'
        
    fileName = preamble + str(runNum)
    return fileName

def GetFailures(runNum,useQuadruped):

    fileName = GetDirName(runNum,useQuadruped)
    print fileName
    
    f = open(fileName,'r')
    failureFile = f.read()
    print failureFile
    
def PrintFailures(damageCase,useQuadruped):

    if not useQuadruped:
        damageCase = damageCase + 20;
        
    for i in range(damageCase,232,40):
        GetFailures(i,useQuadruped)

def ModifyRunScript(scriptNum,sourceRun,newRun):

    oldSeed = sourceRun
    newSeed = newRun
    
    preamble = '..\\DE_' + str(newRun)
    fileName = preamble + '\\run' + str(scriptNum) + '.bat'

    print fileName
    
    f = open(fileName,'r')
    script = f.read()
        
    oldString = '-r ' + str(oldSeed) + ' '
    newString = '-r ' + str(newSeed) + ' '
    script = script.replace(oldString,newString)
    
    f.close()
    f=open(fileName,'w')
    f.write(script)
    f.close()

def CreateNewRuns(sourceRun,endRun):

    for i in range(sourceRun+1,endRun+1):
        CreateNewRun(sourceRun,i)
            
def CreateNewRun(sourceRun,newRun):

        CopyDir(sourceRun,newRun)
        ModifyScript(sourceRun,newRun)
        ModifyRunScript(1,sourceRun,newRun)
        ModifyRunScript(2,sourceRun,newRun)
        ModifyRunScript(3,sourceRun,newRun)
        ModifyRunScript(4,sourceRun,newRun)
                        
def ModifyScript(oldRunNum,runNum):

    oldSeed = oldRunNum
    newSeed = runNum
    
    fileName = GetDamageDirName(runNum)
    fileName = fileName + '\clusterRun.bat'
    print fileName
    f=open(fileName, 'r')
    contents = f.read()
    oldString = 'DE_' + str(oldRunNum)
    newString = 'DE_' + str(runNum)
    contents = contents.replace(oldString,newString)

    oldString = 'run' + str(oldSeed) + '_0_end.dat'
    newString = 'run' + str(newSeed) + '_0_end.dat'
    contents = contents.replace(oldString,newString)

    oldString = 'run' + str(oldSeed) + '_1_end.dat'
    newString = 'run' + str(newSeed) + '_1_end.dat'
    contents = contents.replace(oldString,newString)

    oldString = 'run' + str(oldSeed) + '_2_end.dat'
    newString = 'run' + str(newSeed) + '_2_end.dat'
    contents = contents.replace(oldString,newString)

    #oldString = 'run' + str(oldSeed) + '_3_end.dat'
    #newString = 'run' + str(newSeed) + '_3_end.dat'
    #contents = contents.replace(oldString,newString)
    
    f.close()
    f=open(fileName,'w')
    f.write(contents)
    f.close()
    
def ModifyScripts(startRun,newStart):

    for i in range(startRun,startRun+13):
        ModifyScript(i,newStart)
        newStart = newStart + 1
    
def MakeDirs(sourceRun,destRun):
    sourceName = GetDamageDirName(sourceRun)
    destName = GetDamageDirName(destRun)
    return sourceName, destName

def MoveDataFiles(startRun,endRun):

    import shutil

    for i in range(startRun,endRun+1):

        print i
        
        sourceDir = '..\\Pareto_' + str(i) + '\\data\\'
        targetDir = '..\\lac_Noise\\'

        for j in range(0,1+1):
            sFile = sourceDir + 'run' + str(i) + '_' + str(j) + '_fits.dat'
            dFile = targetDir + 'run' + str(i) + '_' + str(j) + '_fits.dat'
            shutil.copyfile(sFile,dFile);

            sFile = sourceDir + 'run' + str(i) + '_' + str(j) + '_models.dat'
            dFile = targetDir + 'run' + str(i) + '_' + str(j) + '_models.dat'
            shutil.copyfile(sFile,dFile);

            sFile = sourceDir + 'run' + str(i) + '_' + str(j) + '_real.dat'
            dFile = targetDir + 'run' + str(i) + '_' + str(j) + '_real.dat'
            shutil.copyfile(sFile,dFile);

            sFile = sourceDir + 'run' + str(i) + '_' + str(j) + '_runState.dat'
            dFile = targetDir + 'run' + str(i) + '_' + str(j) + '_runState.dat'
            shutil.copyfile(sFile,dFile);

            sFile = sourceDir + 'run' + str(i) + '_' + str(j) + '_tests.dat'
            dFile = targetDir + 'run' + str(i) + '_' + str(j) + '_tests.dat'
            shutil.copyfile(sFile,dFile);
            
        sourceDir = '..\\Gait_' + str(i) + '\\data\\'
        targetDir = '..\\lac_Noise\\'

        for j in range(2,3+1):
            sFile = sourceDir + 'run' + str(i) + '_' + str(j) + '_fits.dat'
            dFile = targetDir + 'run' + str(i) + '_' + str(j) + '_fits.dat'
            shutil.copyfile(sFile,dFile);

            sFile = sourceDir + 'run' + str(i) + '_' + str(j) + '_models.dat'
            dFile = targetDir + 'run' + str(i) + '_' + str(j) + '_models.dat'
            shutil.copyfile(sFile,dFile);

            sFile = sourceDir + 'run' + str(i) + '_' + str(j) + '_real.dat'
            dFile = targetDir + 'run' + str(i) + '_' + str(j) + '_real.dat'
            shutil.copyfile(sFile,dFile);

            sFile = sourceDir + 'run' + str(i) + '_' + str(j) + '_runState.dat'
            dFile = targetDir + 'run' + str(i) + '_' + str(j) + '_runState.dat'
            shutil.copyfile(sFile,dFile);
        
def CopyDir(sourceRun,destRun):
    import shutil
    sourceName, destName = MakeDirs(sourceRun,destRun)
    print sourceName
    print destName
    shutil.copytree(sourceName,destName)

def FixBothStrings(runNum):

    FixDataFile(runNum,';-1.#IND')
    FixDataFile(runNum,';1.#QNAN')
    
def FixDataFile(runNum,oldString):

    newString = ';0'
    
    fileName = 'C:\\PostDoc\\failure\\MoreCases\\Metrics'
    fileName = fileName + '\\Metrics_'+ str(runNum) + '\\data\\run' + str(runNum) + '_fit.dat'
    print fileName
    f=open(fileName,'r')
    contents = f.read()
    contents = contents.replace(oldString,newString)
    f.close()
    f=open(fileName,'w')
    f.write(contents)
    f.close()
    
    fileName = 'C:\\PostDoc\\failure\\MoreCases\\Metrics'
    fileName = fileName + '\\Metrics_'+ str(runNum) + '\\data\\run' + str(runNum+40) + '_fit.dat'
    print fileName
    f=open(fileName,'r')
    contents = f.read()
    contents = contents.replace(oldString,newString)
    f.close()
    f=open(fileName,'w')
    f.write(contents)
    f.close()

    fileName = 'C:\\PostDoc\\failure\\MoreCases\\Metrics'
    fileName = fileName + '\\Metrics_'+ str(runNum) + '\\data\\run' + str(runNum+80) + '_fit.dat'
    print fileName
    f=open(fileName,'r')
    contents = f.read()
    contents = contents.replace(oldString,newString)
    f.close()
    f=open(fileName,'w')
    f.write(contents)
    f.close()

    fileName = 'C:\\PostDoc\\failure\\MoreCases\\Metrics'
    fileName = fileName + '\\Metrics_'+ str(runNum) + '\\data\\run' + str(runNum+120) + '_fit.dat'
    print fileName
    f=open(fileName,'r')
    contents = f.read()
    contents = contents.replace(oldString,newString)
    f.close()
    f=open(fileName,'w')
    f.write(contents)
    f.close()
    
def FixDataFiles(startRun,endRun):

    for i in range(startRun,endRun+1):
        FixBothStrings(i)

def ExtractRun(runNum,extension):

    fileName = 'C:\\PostDoc\\Papers\\2004_EC\\FSM\\code\\FSM_'
    fileName = fileName + str(runNum) + '\\data\\run' + str(runNum) + '_0_' + extension

    newFileName = 'C:\\PostDoc\\Papers\\2004_EC\\FSM'
    newFileName = newFileName + '\\data\\run' + str(runNum) + '_0_' + extension

    f=open(fileName,'r')
    contents = f.read()
    f.close()
    f=open(newFileName,'w')
    f.write(contents)
    f.close()
        
    print fileName
    print newFileName

    fileName = 'C:\\PostDoc\\Papers\\2004_EC\\FSM\\code\\FSM_'
    fileName = fileName + str(runNum) + '\\data\\run' + str(runNum) + '_1_' + extension

    newFileName = 'C:\\PostDoc\\Papers\\2004_EC\\FSM'
    newFileName = newFileName + '\\data\\run' + str(runNum) + '_1_' + extension

    f=open(fileName,'r')
    contents = f.read()
    f.close()
    f=open(newFileName,'w')
    f.write(contents)
    f.close()
    
    print fileName
    print newFileName
    
def ExtractRuns(startRun,endRun):

    for i  in range(startRun,endRun+1):
        ExtractRun(i,'fit.dat')
        ExtractRun(i,'best.dat')
        ExtractRun(i,'experiments.dat')

def FixAndExtract(startRun,endRun):
    FixDataFiles(startRun,endRun)
    ExtractRuns(startRun,endRun)
