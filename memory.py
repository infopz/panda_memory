import random
from random import randint

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

class Memory:
    def __init__(self, cubes, similarityFunc):
        self.cubes = cubes
        self.cubesNumber = len(self.cubes)
        self.similairty = similarityFunc
        self.userPoint = 0
        self.robotPoint = 0

    def chooseRandom(self):
        # Choose a random cube, if none, return None

        available = [i for i in range(self.cubesNumber) if self.cubes[i].status != 2]
        if len(available) == 0:
            return None
        else:
            return random.choice(available)

    def findCouple(self):
        # Find a pair of cubes

        for i in range(self.cubesNumber):
            if self.cubes[i].status != 1:
                # 1 is the status of cube viewed
                continue
            for j in range(self.cubesNumber):
                if j==i or self.cubes[j].states != 1:
                    continue

                if self.similairty(self.cubes[i].description, self.cubes[j].description):
                    # if a couple is found, return their index
                    return i, j
        else:
            return None, None

    def findMatch(self, first):
        # Same as findCouple but takes the first cube index as input

        for j in range(self.cubesNumber):
            if j == first or self.cubes[j].states != 1:
                continue

            if self.similairty(self.cubes[first].description, self.cubes[j].description):
                return j
        else:
            return None