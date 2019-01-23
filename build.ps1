# setup env
$env:Path += ";c:\Program Files (x86)\Atmel\Studio\7.0\"

$projectFile = ".\SAMDx1-dfu\SAMx-dfu.cproj"
$buildType = "Release"

# check if working dir is clean
git diff-index --quiet HEAD
if ($LASTEXITCODE -ne 0) {
  # error not clean
  echo "Working-dir not clean."
  #Read-Host -Prompt "Press Enter to exit"
  #exit
}

# get version from git
$gitVer = (git rev-parse --short HEAD)
$version = ((Get-Date -UFormat %Y%m%d) + "_git-" + $gitVer)
$builddir = ".\build-" + $version

# prepare build dir
$buildDir = ".\_build\" + $version + "\"
$buildLog = "build-log.txt"
if (Test-Path $buildDir) {
  Remove-Item -Recurse -Force $buildDir
}
New-Item -ItemType directory -Path $buildDir | Out-Null
# create a log of the recent git commits
git log -10 > ($buildDir + "\git-commits.log")

# remove old build log
if (Test-Path $buildLog) {
  Remove-Item $buildLog
}

# put version into project file
$search  = 'GIT_SHORT_TAG=\"develop\"'
$replace = 'GIT_SHORT_TAG=\"' + $gitVer + '\"'
(Get-Content $projectFile).replace($search, $replace) | Set-Content $projectFile

# start the build
Write-Host "Running Build " -NoNewLine
AtmelStudio.exe SAMx-dfu.atsln /out $buildLog /rebuild $buildType /project SAMx-dfu

# wait for the build to finish
while (!(Test-Path $buildLog)) { Start-Sleep 1 }
while (!([string](Get-Content $buildLog) -match  '========== Rebuild All: (\d) succeeded, (\d) failed, (\d) skipped ==========')) { 
  Start-Sleep 1 
  Write-Host "." -NoNewLine
}
Write-Host "."
$BuildNumOK = [int]$matches[1]
$BuildNumFail = [int]$matches[2]
$BuildNumSkip = [int]$matches[3]


if ($BuildNumOK.Equals(1)) {
  echo "Build Okay!"
  # copy to build dir
  Copy-Item (".\SAMDx1-dfu\" + $buildType + "\SAMx-dfu.bin") ($buildDir + $version + "_SAMx-dfu.bin")
  Copy-Item (".\SAMDx1-dfu\" + $buildType + "\SAMx-dfu.map") ($buildDir + $version + "_SAMx-dfu.map")
  Copy-Item (".\SAMDx1-dfu\" + $buildType + "\SAMx-dfu.elf") ($buildDir + $version + "_SAMx-dfu.elf")
  Copy-Item (".\SAMDx1-dfu\" + $buildType + "\SAMx-dfu.lss") ($buildDir + $version + "_SAMx-dfu.lss")
  Copy-Item $buildLog $buildDir
}

if ($BuildNumFail.Equals(1)) {
  echo "Build Failed!"
}else {
  Remove-Item $buildLog
}

# cleanup
git checkout $projectFile

# done :)
Read-Host -Prompt "Press Enter to exit"