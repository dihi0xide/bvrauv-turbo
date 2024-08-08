# import matplotlib.pyplot as plt
# import PID
#
# # just a test environment to make sure the PID is pretty much working.
# # this isn't a replacement for water testing! you still need test in an actual scenario
# # to tune the parameters, this is pretty minor
#
#
# class WaterSimulation:
#     def __init__(self, pid: PID):
#         self.pid: PID = pid
#         self.vel: float = 0
#         self.pos: float = 0
#         self.min: float = 0
#         self.gravity: float = 9.8
#         self.damping: float = 2
#
#     def tick(self) -> None:
#         """Update the simulation by one tick"""
#         self.vel -= self.gravity
#         self.vel /= self.damping
#         signal = self.pid.predict(self.pos)
#         self.vel += signal
#         self.pos += self.vel
#
#     def test_and_display(self, iterations: int):
#         """Tests the PID over `iterations` iterations, and plots it to plt"""
#         positions = []
#         velocities = []
#         for i in range(iterations):
#             self.tick()
#             positions.append(self.pos)
#             velocities.append(self.vel)
#
#         plt.plot(range(iterations), positions)
#
#
# sim = WaterSimulation(PID.PID(0.6, 0.0, 0.1, 20, 0.1))
# sim.test_and_display(100)
#
# plt.savefig("pid_test.png")
# # this will be made better
# # or maybe not, ha
