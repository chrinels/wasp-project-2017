# WASP Project course 2017
Group members:
- Academic supervisor: Paolo Falcone (Chalmers)
- Industrial supervisor: Henrik Sahlin (Ericsson)
- Industrial supervisor: Björn Löfdahl (Ericsson)
- Pavel Anistratov (Linköping University, LiU)
- Victor Fors (Linköping University, LiU)
- Vidit Saxena (Royal Institute of Technology, KTH)
- Christian Nelson (Lund University, LTH) 

## Install

Follwing commands will remove all docker images and containers on your computer
```
cd ~

docker kill $(docker ps -q)
docker rm $(docker ps -a -q)
docker rmi -f $(docker images -q)

git clone git@github.com:chrinels/wasp-project-2017.git
git submodule update --init

cd ~/wasp-project-2017/opendlv.core/docker
make buildComplete createDockerImage

cd ~/wasp-project-2017/opendlv/docker
make buildComplete createDockerImage

```

## Links
- [HackMD.io](https://hackmd.io/MYEwnADALATBDMBaYBDFFFQEYoGaIA4BGLAU0TCwFYB2EeEFANixviA=)
- [MIT Senseable City Lab - lightTraffic](http://senseable.mit.edu)
- [COPPLAR](https://www.saferresearch.com/projects/copplar-campusshuttle-cooperative-perception-and-planning-platform)
