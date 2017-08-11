#######################################
### RUNNING A [hello world] SOAR AGENT IN PYTHON
#######################################

python3 p4jSoarAgent.py


#######################################
### COMPILING THE JAVA SOAR INSTANTIATION
### (Only needed if you modify SoarAgent.java
#######################################


javac -cp ".;*;" P4jSoarAgent.java

or in linux

javac -cp ".:*:" P4jSoarAgent.java

Check if ok by running:

java -cp ".;*;" P4jSoarAgent hello-world-simple.soar

or in linux

java -cp ".:*:" P4jSoarAgent hello-world-simple.soar


