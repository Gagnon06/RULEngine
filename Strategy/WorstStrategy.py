from Command import Command
from Strategy.Strategy import Strategy
from Util.Position import Position


class WorstStrategy(Strategy):
    def __init__(self, field, referee, team, opponent_team):
        super().__init__(field, referee, team, opponent_team)

    def update(self):
        self._send_command(Command.MoveTo(self.team.players[0], Position(500, 100, 0)))
