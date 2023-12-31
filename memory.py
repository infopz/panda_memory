import random
import time
import telegram


class Memory:
    # Defines the behaviour of the entire game and the robot actions

    def __init__(self, cubes, similarityFunc):
        # Takes as input the cubes and a similarity function to compare thier descriptors

        self.cubes = cubes
        self.cubesNumber = len(self.cubes)
        self.similairty = similarityFunc

        self.humanPoint = 0
        self.robotPoint = 0
        #self.turn = random.choice([0, 1])  # 0 robot, 1 human
        self.turn = 1

    def play(self):
        # Starts the game

        telegram.send_message("Welcome to the PandaMemory game!")

        end = False
        while not end:
            if self.turn == 0:
                self.robot_turn()
            else:
                self.human_turn()

            end = all([i.status == 2 for i in self.cubes])  # If all cubes are in state 2
            if not end:
                telegram.send_message("Current point\nH: " + str(self.humanPoint) + " R: " +str(self.robotPoint))

        end_msg = "No more cubes are available.\nThe match is ended with a score of:\n"
        end_msg += "H: " + str(self.humanPoint) + " R: " + str(self.robotPoint) +"\n"
        if self.humanPoint > self.robotPoint:
            end_msg += "The winner is the human!"
        elif self.humanPoint < self.robotPoint:
            end_msg += "The winner is the Robot!"
        else:
            end_msg += "It's a tie!"

        telegram.send_message(end_msg)

    def robot_turn(self):
        both_random = False
        # This became true if both the cubes are picked at random
        # and so we have to check the similairty

        # First search for a possibile couple, otherwhise choose a random cube
        i, j = self.findCouple()
        if i is None:
            # No available known couple found
            i = self.chooseRandom()
            if i is None:
                # No available cubes, match ended
                return False

        telegram.send_message("It's my turn, I choose the cube " + str(i))
        self.cubes[i].pick_view_place()

        # Then select the other cube, if available, for the previous findCouple
        if j is None:
            # No couple find at the start of the turn
            j = self.findMatch(i)
            if j is None:
                # Still no couple found for the first cube
                j = self.chooseRandom()
                both_random = True

        telegram.send_message("Then I choose " + str(j))
        self.cubes[j].pick_view_place()

        # Check if the cubes have the same image
        if both_random == True and not self.similairty(self.cubes[i].description, self.cubes[j].description):
            # Not the same couple
            telegram.send_message("Oh no, couple not found. It's your turn.")
            self.turn = 1
        else:
            # Couple found for a previous match or because similarity==True
            telegram.send_message("Couple found! Now I will put this cubes away.")

            self.cubes[i].put_away(0)
            time.sleep(1)
            self.cubes[j].put_away(0)

            self.robotPoint += 1

    def human_turn(self):
        # The repeat is used to check if the choose cube is in state 0 or 1

        repeat = True
        while repeat:
            telegram.send_message("Which is the first cube you want to see?", keyboard=self.cubesNumber)
            i = int(telegram.receive_message())
            repeat = self.cubes[i].status == 2
            if repeat:
                telegram.send_message("The choose cube is not available!")

        self.cubes[i].pick_view_place()

        repeat = True
        while repeat:
            telegram.send_message("Now give me the number of the other cube", keyboard=self.cubesNumber)
            j = int(telegram.receive_message())
            repeat = self.cubes[j].status == 2 or j == i
            if repeat:
                telegram.send_message("The choosed cube is not available!")

        self.cubes[j].pick_view_place()

        if self.similairty(self.cubes[i].description, self.cubes[j].description):
            telegram.send_message("Nice! You found a couple!\nI will put those cubes away.")
            self.cubes[i].put_away(1)
            time.sleep(1)
            self.cubes[j].put_away(1)

            self.humanPoint += 1
        else:
            telegram.send_message("Oh no, it's not a couple")
            self.turn = 0

    def chooseRandom(self):
        # Choose a random cube, giving priority to those not seen

        not_viewed = [i for i in range(self.cubesNumber) if self.cubes[i].status == 0]
        if len(not_viewed) == 0:
            # Chose a random avalable cube
            available = [i for i in range(self.cubesNumber) if self.cubes[i].status != 2]
            if len(available) == 0:
                # Game Ended (not reachable in theory)
                return None
            else:
                return random.choice(available)
        else:
            return random.choice(not_viewed)

    def findCouple(self):
        # Find a pair of cubes

        for i in range(self.cubesNumber):
            if self.cubes[i].status != 1:
                # 1 is the status of cube viewed
                continue
            for j in range(self.cubesNumber):
                if j == i or self.cubes[j].status != 1:
                    continue

                if self.similairty(self.cubes[i].description, self.cubes[j].description):
                    # if a couple is found, return their index
                    return i, j
        else:
            return None, None

    def findMatch(self, first):
        # Same as findCouple but takes the first cube index as input

        for j in range(self.cubesNumber):
            if j == first or self.cubes[j].status != 1:
                continue

            if self.similairty(self.cubes[first].description, self.cubes[j].description):
                return j
        else:
            return None


def test_memory():
    cube_values = [1, 2, 3, 4, 1, 2, 3, 4]

    class FakeCube:
        def __init__(self, id):
            # 0 not_viewed, 1 viewed, 2 used
            self.status = 0
            self.description = None
            self.myID = id

        def pick_view_place(self):
            self.description = cube_values[self.myID]
            self.status = 1
            print("Cube ", self.myID, "picked and viewed")

        def put_away(self):
            self.status = 2
            print("Cube ", self.myID, "put away")

    cubes = [FakeCube(i) for i in range(8)]
    m = Memory(cubes, lambda x, y: x == y)
    m.play()


if __name__ == "__main__":
    test_memory()


''' Old Memory Code from 2017
class MyClass(GeneratedClass):
    def __init__(self):
        GeneratedClass.__init__(self)
        self.arr = [0, 0, 0, 0, 0, 0, 0, 0]
        self.tro = [0, 0, 0, 0, 0, 0, 0, 0]
        self.posCorr = 0
        self.fase = 0
        self.primoNum = 0
        self.stop = 0
        self.coppieUser = 0
        self.coppieNao = 0

    def onLoad(self):
        # put initialization code here
        pass

    def onUnload(self):
        # put clean-up code here
        pass

    def onInput_inputStop(self):
        self.stop=1
        self.outputStop()

    def onInput_InputUser(self, p):
        if self.stop == 1:
            return
        if len(p)!=2:
            return
        self.posCorr = p[0]
        self.arr[self.posCorr] = p[1]
        if self.fase == 2:
            self.primoNum = self.posCorr
            self.fase = 3
            self.outputUser("Bene, ora gira la seconda carta, dimmi che numero scegli")
        elif self.fase == 3:
            if self.arr[self.posCorr] == self.arr[self.primoNum]:
                self.tro[self.posCorr] = 1
                self.tro[self.primoNum] = 1
                self.coppieUser += 1
                self.posCorr = self.cercaCoppia()
                if self.posCorr != 1205:
                    self.fase = 0
                    self.outputNao("Complimenti, hai trovato la coppia! Ora tocca a me, girami il numero "+str(self.posCorr))
                else:
                    self.posCorr = self.getNextR()
                    if self.posCorr != 1205:
                        self.fase = 0
                        self.outputNao("Complimenti, hai trovato la coppia! Ora tocca a me, girami il numero " + str(self.posCorr))
                    else:
                        self.oFine(self.cercaVincita())
            else:
                self.posCorr = self.cercaCoppia()
                if self.posCorr != 1205:
                    self.fase = 0
                    self.outputNao("Questo turno sei stato sfortunato, ora tocca a me, girami il numero " + str(self.posCorr))
                else:
                    self.posCorr = self.getNextR()
                    if self.posCorr != 1205:
                        self.fase = 0
                        self.outputNao("Questo turno sei stato sfortunato, ora tocca a me, girami il numero " + str(self.posCorr))
                    else:
                        self.oFine(self.cercaVincita()) #non dovrebbe accadere

    def onInput_onStart(self, f):
        self.outputNao("E' ora del memory! Comincio io, mostrami il numero 0")

    def onInput_onStop(self):
        self.onUnload()  # it is recommended to reuse the clean-up as the box is stopped
        self.outputNao()  # activate the output of the box

    def onInput_InputNao(self, p):
        if self.stop == 1:
            return
        try:
            self.arr[self.posCorr] = p
        except IndexError:
            self.oFine(self.cercaVincita())
        if self.fase == 0:
            self.primoNum = self.posCorr
            self.posCorr = self.checkNextSame()
            if self.posCorr != 1205:
                self.fase = 1
                self.outputNao("Mi sento fiducioso, girami il numero "+str(self.posCorr))
            else:
                self.posCorr = self.getNextR()
                if self.posCorr != 1205:
                    self.fase = 1
                    self.outputNao("Bene, ora mostrami il numero "+str(self.posCorr))
                else:
                    self.oFine(self.cercaVincita()) #mai succede
        elif self.fase == 1:
            if self.arr[self.posCorr] == self.arr[self.primoNum]:
                self.tro[self.posCorr] = 1
                self.tro[self.primoNum] = 1
                self.coppieNao += 1
                if not self.tuttiTrovati():
                    self.fase = 2
                    self.outputUser("Ottimo, sono bravissimo, ora e' il tuo turno, dimmi che numero scegli")
                else:
                    self.oFine(self.cercaVincita())
            else:
                if not self.tuttiTrovati():
                    self.fase = 2
                    self.outputUser("Questo turno mi e' andata male, ora e' il tuo turno, dimmi che numero scegli")
                else:
                    self.oFine(self.cercaVincita()) #mai succede



    def getNextR(self):
        for i in range(0, len(self.arr)):
            r = randint(0, 7)
            if self.arr[r] == 0:
                return r
        return 1205

    def tuttiTrovati(self):
        for i in self.tro:#FANCULO
            if i != 1:
                return False
        return True

    def cercaCoppia(self):
        for i in range(0, len(self.arr)):
            for j in range(0, len(self.arr)):
                if self.arr[j] == self.arr[i] and j != i and self.tro[j] != 1:
                    return i
        return 1205

    def cercaVincita(self):
        if self.coppieNao > self.coppieUser:
            punteggio = str(self.coppieNao) + " a " + str(self.coppieUser)
            return "Che bello, ho vinto "+punteggio+", e' stata una bella partita, ora mettiamo a posto!"
        elif self.coppieNao < self.coppieUser:
            punteggio = str(self.coppieUser) + " a " + str(self.coppieNao)
            return "Complimenti, hai vinto"+punteggio+", e' stata una bella partita ora mettiamo a posto!"
        else:
            punteggio = str(self.coppieNao) + " a " + str(self.coppieUser)
            return "Abbiamo pareggiato "+punteggio+", ma ci siamo divertiti! Ora mettiamo a posto!"

    def checkNextSame(self):
        for i in range(0, len(self.arr)):  # ciclo per trovare eventuale coppia
            if self.arr[i] == self.arr[self.posCorr] and i != self.posCorr:
                return i
        return 1205
'''