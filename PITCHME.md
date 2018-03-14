# Motion Profiling Code Survey 

A Survey of MP Code Around the Web

---

### Team 135 Penn Robotics

- Uses Pathfinder Library
- Gens Path/Trajectory on the fly (with limited success)
- Feeds trajectory points to motors using Command subsystem
- Yanked from [Chief Delphi](https://www.chiefdelphi.com/forums/showthread.php?p=1745136 "Robot Code Never Starts")

+++?code=DriveAlongProfile.java&lang=java

@[11-13](Uses Jaci Pathfinder library)
@[17](Implemented as WPILIB Command...useful)
@[28](50ms trajectory timing)
@[27-31](Ticks converted to meters...later converted into inches...ich!)
@[33](Normalizes power within setPoint range of -1..1)
@[35-39](Trajectory point feedback PID constants...note large D dampending...hmmm)
@[45-49](Heading correction PID constants)
@[53-54](Timer simply used to timout command)


###### Team 135: [DriveAlongProfile.java](https://github.com/Team997Coders/MPNotes/blob/master/DriveAlongProfile.java)
