#Smooths a value while taking its derivative with respect to time.
smoothDerivative <- function(value, timeMillis, n){
  smoothed <- (value[(n+1):length(value)] - value[1:(length(value)-n)])/((timeMillis[(n+1):length(timeMillis)] - timeMillis[1:(length(timeMillis)-n)])/1000);
  return(c(rep(0, ceiling(n/2)), smoothed, rep(0, floor(n/2))));
}

characterize <- function(velFile, accelFile, smoothing = 2){
  vel <- read.csv(velFile)
  accel <- read.csv(accelFile)
  goodVel <- subset(vel, abs(LeftEncoderSpeed) > 0.2 & LeftMasterVoltage!=0 & abs(RightEncoderSpeed) > 0.2 & RightMasterVoltage!=0)
  goodVel$LeftAccel <- smoothDerivative(goodVel$LeftEncoderSpeed, goodVel$Time, smoothing)
  goodVel$RightAccel <- smoothDerivative(goodVel$RightEncoderSpeed, goodVel$Time, smoothing)
  accel$LeftAccel <- smoothDerivative(accel$LeftEncoderSpeed, accel$Time, smoothing)
  accel$RightAccel <- smoothDerivative(accel$RightEncoderSpeed, accel$Time, smoothing)
  goodAccel <- subset(accel, LeftMasterVoltage != 0 & RightMasterVoltage != 0)
  goodAccelLeft <- goodAccel[(which.max(abs(goodAccel$LeftAccel))+1):length(goodAccel$Time),]
  goodAccelRight <- goodAccel[(which.max(abs(goodAccel$RightAccel))+1):length(goodAccel$Time),]
  combinedLeftVoltage <- c(goodVel$LeftMasterVoltage, goodAccelLeft$LeftMasterVoltage)
  combinedRightVoltage <- c(goodVel$RightMasterVoltage, goodAccelRight$RightMasterVoltage)
  combinedLeftVel <- c(goodVel$LeftEncoderSpeed, goodAccelLeft$LeftEncoderSpeed)
  combinedRightVel <- c(goodVel$RightEncoderSpeed, goodAccelRight$RightEncoderSpeed)
  combinedLeftAccel <- c(goodVel$LeftAccel, goodAccelLeft$LeftAccel)
  combinedRightAccel <- c(goodVel$RightAccel, goodAccelRight$RightAccel)
  plot(goodAccelLeft$Time, goodAccelLeft$LeftAccel)
  plot(goodVel$Time, goodVel$LeftMasterVoltage)
  plot(goodVel$LeftMasterVoltage, goodVel$LeftEncoderSpeed)
  plot(goodAccelRight$Time, goodAccelRight$RightAccel)
  plot(goodVel$Time, goodVel$RightMasterVoltage)
  plot(goodVel$RightMasterVoltage, goodVel$RightEncoderSpeed)
  leftModel <- lm(combinedLeftVoltage~combinedLeftVel+combinedLeftAccel)
  rightModel <- lm(combinedRightVoltage~combinedRightVel+combinedRightAccel)
  print(summary(leftModel))
  print(summary(rightModel))
}