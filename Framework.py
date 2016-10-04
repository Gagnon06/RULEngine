# Under MIT License, see LICENSE.txt
"""
    Point de départ du moteur pour l'intelligence artificielle. Construit les
    objets nécessaires pour maintenir l'état du jeu, acquiert les frames de la
    vision et construit la stratégie. Ensuite, la stratégie est exécutée et un
    thread est lancé qui contient une boucle qui se charge de l'acquisition des
    frames de la vision. Cette boucle est la boucle principale et appel le
    prochain état du **Coach**.
"""
from collections import namedtuple
import signal
import threading
import time

from .Game.Game import Game
from .Game.Referee import Referee
from .Communication.vision import Vision
from .Communication.referee import RefereeServer
from .Communication.udp_server import GrSimCommandSender, DebugCommandSender,\
                                      DebugCommandReceiver, GrSimDebugSender
from .Communication.serial_command_sender import SerialCommandSender
from .Command.command import Stop
from .Util.exception import StopPlayerError

LOCAL_UDP_MULTICAST_ADDRESS = "224.5.23.2"
UI_DEBUG_MULTICAST_ADDRESS = "127.0.0.1"

CMD_DELTA_TIME = 0.030

GameState = namedtuple('GameState', ['field', 'referee', 'friends',
                                     'enemies', 'timestamp', 'debug'])

class Framework(object):
    """
        La classe contient la logique nécessaire pour démarrer une logique et
        mettre en place l'état du jeu.
    """

    def __init__(self, is_team_yellow=False):
        """ Constructeur de la classe, établis les propriétés de bases. """
        # TODO: refactor pour avoir des objets qui contiennent les petits
        # détails d'implémentations (12/7 fields)
        self.command_sender = None
        self.debug_sender = None
        self.debug_receiver = None
        self.grsim_syncer = None
        self.game = None
        self.is_team_yellow = is_team_yellow
        self.ai_coach = None
        self.referee = None
        self.running_thread = False
        self.last_frame_number = 0
        self.thread_terminate = threading.Event()
        self.times = 0
        self.last_time = 0
        self.vision = None
        self.last_cmd_time = time.time()


    def create_game(self, ai_coach):
        """
            Créé l'arbitre et établit la stratégie de l'équipe que l'ia gère.

            :param ai_coach: Une référence vers une stratégie non instanciée.
            :return: Game, le **GameState**
        """

        self.referee = Referee()

        self.ai_coach = ai_coach()

        self.game = Game(self.referee, self.is_team_yellow)

        return self.game


    def update_game_state(self):
        """ Met à jour le **GameState** selon la vision et l'arbitre. """
        # TODO: implémenter correctement la méthode
        pass

    def update_players_and_ball(self, vision_frame):
        """ Met à jour le GameState selon la frame de vision obtenue. """
        time_delta = self._compute_vision_time_delta(vision_frame)
        self.game.update(vision_frame, time_delta)

    def _is_frame_number_different(self, vision_frame):
        if vision_frame is not None:
            return vision_frame.detection.frame_number != self.last_frame_number
        else:
            return False

    def _compute_vision_time_delta(self, vision_frame):
        self.last_frame_number = vision_frame.detection.frame_number
        this_time = vision_frame.detection.t_capture
        time_delta = this_time - self.last_time
        self.last_time = this_time
        # FIXME: hack
        # print("frame: %i, time: %d, delta: %f, FPS: %d" % \
        #        (vision_frame.detection.frame_number, this_time, time_delta, 1/time_delta))
        return time_delta

    def update_strategies(self):
        """ Change l'état du **Coach** """

        game_state = self.get_game_state()

        # FIXME: il y a probablement un refactor à faire avec ça
        # state = self.referee.command.name
        state = "NORMAL_START"
        if state == "NORMAL_START":
            self.ai_coach.main_loop(game_state)

        elif state == "STOP":
            self.ai_coach.stop(game_state)


    def get_game_state(self):
        """ Retourne le **GameState** actuel. *** """

        game = self.game
        return GameState(
            field=game.field,
            referee=game.referee,
            friends=game.friends,
            enemies=game.enemies,
            timestamp=self.last_time,
            debug=self.debug_receiver.receive_command()
        )

    def start_game(self, ai_coach, async=False, serial=False):
        """ Démarrage du moteur de l'IA initial. """

        # on peut eventuellement demarrer une autre instance du moteur
        # TODO: method extract -> _init_communication_serveurs()
        if not self.running_thread:
            if serial:
                self.command_sender = SerialCommandSender()
            else:
                self.command_sender = GrSimCommandSender(LOCAL_UDP_MULTICAST_ADDRESS, 20011)

            self.debug_sender = DebugCommandSender(UI_DEBUG_MULTICAST_ADDRESS, 20021)
            self.debug_receiver = DebugCommandReceiver(UI_DEBUG_MULTICAST_ADDRESS, 10021)
            self.referee = RefereeServer(LOCAL_UDP_MULTICAST_ADDRESS)
            self.vision = Vision(LOCAL_UDP_MULTICAST_ADDRESS)

            self.grsim_syncer = GrSimDebugSender(LOCAL_UDP_MULTICAST_ADDRESS, 20011)
        else:
            self.stop_game()

        self.create_game(ai_coach)

        signal.signal(signal.SIGINT, self._sigint_handler)
        self.running_thread = threading.Thread(target=self.game_thread_main_loop)
        self.running_thread.start()

        if not async:
            self.running_thread.join()

    def game_thread_main_loop(self):
        """ Fonction exécuté et agissant comme boucle principale. """

        self._wait_for_first_frame()

        # TODO: Faire arrêter quand l'arbitre signal la fin de la partie
        while not self.thread_terminate.is_set():
            # TODO: method extract
            # Mise à jour
            current_vision_frame = self._acquire_vision_frame()
            if self._is_frame_number_different(current_vision_frame):
                self.update_game_state()
                self.update_players_and_ball(current_vision_frame)
                self.update_strategies()

                # Communication
                self._send_robot_commands()
                self._send_debug_commands()

    def _acquire_vision_frame(self):
        return self.vision.get_latest_frame()

    def stop_game(self):
        """
            Nettoie les ressources acquises pour pouvoir terminer l'exécution.
        """
        self.thread_terminate.set()
        self.running_thread.join()
        self.thread_terminate.clear()
        try:
            if self.is_team_yellow:
                team = self.game.yellow_team
            else:
                team = self.game.blue_team
            for player in team.players:
                command = Stop(player)
                self.command_sender.send_command(command)
        except:
            print("Could not stop players")
            raise StopPlayerError("Au nettoyage il a été impossible d'arrêter\
                                    les joueurs.")

    def _wait_for_first_frame(self):
        while not self.vision.get_latest_frame():
            time.sleep(0.01)
            print("En attente d'une image de la vision.")

    def _send_robot_commands(self):
        cmd_time = time.time()
        if cmd_time - self.last_cmd_time > CMD_DELTA_TIME:
            self.last_cmd_time = cmd_time
            commands = self._get_coach_robot_commands()
            self.grsim_syncer.send_command(commands[4])
            commands[4] = commands[4].to_speed_command()
            commands[4].pose.orientation = 0
            self.command_sender.send_command(commands[4])

            #for command in commands:
            #    command = command.to_speed_command()
            #    command.pose.orientation = 0
            #    self.command_sender.send_command(command)


    def _get_coach_robot_commands(self):
        return self.ai_coach.robot_commands


    def _send_debug_commands(self):
        """ Récupère les paquets de débogages et les envoies au serveur. """
        ai_debug_commands = self.ai_coach.get_debug_commands_and_clear()
        if ai_debug_commands:
            self.debug_sender.send_command(ai_debug_commands)


    def _sigint_handler(self, signum, frame):
        self.stop_game()
