from gym.envs.registration import register
register(
    id = 'PandaJuggling-v0',
    entry_point = 'Panda_Juggling.envs:PandaJugglingEnv' 
)