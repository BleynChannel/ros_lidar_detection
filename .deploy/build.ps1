New-Item -ItemType Directory -Force -Path $PSScriptRoot\build

Copy-Item -Path $PSScriptRoot\docker-compose.yml -Destination $PSScriptRoot\build

docker build -t bleyn/3d_detection:latest $PSScriptRoot\..
docker image save bleyn/3d_detection:latest -o $PSScriptRoot\build\3d_detection.tar

"docker load < $PSScriptRoot\3d_detection.tar
docker-compose -f $PSScriptRoot\docker-compose.yml up -d" | Out-File -FilePath $PSScriptRoot\build\deploy.ps1