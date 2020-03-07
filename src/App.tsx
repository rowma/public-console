import React, { useEffect } from 'react';
import './App.css';

import { createMuiTheme, makeStyles, ThemeProvider, Theme, createStyles } from '@material-ui/core/styles';

import AppBar from '@material-ui/core/AppBar';
import Container from '@material-ui/core/Container';
import Toolbar from '@material-ui/core/Toolbar';
import Typography from '@material-ui/core/Typography';
import Paper from '@material-ui/core/Paper';
import Grid from '@material-ui/core/Grid';
import Box from '@material-ui/core/Box';
import Link from '@material-ui/core/Link';

import Radio from '@material-ui/core/Radio';
import RadioGroup from '@material-ui/core/RadioGroup';
import FormControlLabel from '@material-ui/core/FormControlLabel';
import FormControl from '@material-ui/core/FormControl';
import Button from '@material-ui/core/Button';

import Table from '@material-ui/core/Table';
import TableBody from '@material-ui/core/TableBody';
import TableCell from '@material-ui/core/TableCell';
import TableContainer from '@material-ui/core/TableContainer';
import TableRow from '@material-ui/core/TableRow';

// @ts-ignore
import Rowma from 'rowma_js';

const theme = createMuiTheme({
  palette: {
    primary: {
      main: '#ffffff',
    },
    secondary: {
      main: '#ED4A70',
    },
    contrastThreshold: 3,
    tonalOffset: 0.2,
  },
  typography: {
    button: {
      textTransform: 'none'
    }
  }
});

const useStyles = makeStyles((theme: Theme) => (
  createStyles({
    root: {
      flexGrow: 1,
    },
    menuButton: {
      marginRight: theme.spacing(2),
    },
    title: {
      flexGrow: 1,
    },
    paper: {
      padding: theme.spacing(2),
      textAlign: 'center',
      color: theme.palette.text.secondary,
      background: '#fdfdfd'
    },
    radioButtons: {
      maxHeight: 300,
      minHeight: 300,
      textAlign: 'center',
      overflow: 'auto'
    },
    info: {
      padding: theme.spacing(2),
      textAlign: 'center',
      color: theme.palette.text.secondary,
      background: '#f6f6f6'
    },
    header: {
      color: theme.palette.text.primary,
      background: '#fcfcfc'
    },
    footer: {
      textAlign: 'left',
    },
    footerLink: {
      color: '#38B48B',
      marginRight: '1rem',
    },
    radioGroup: {
      textAlign: 'left',
    }
  })
));

const rowma = new Rowma({ baseURL: 'https://rocky-peak-54058.herokuapp.com' });

const App: React.FC = () => {
  const [robotUuids, setRobotUuids] = React.useState<Array<string> | undefined>(undefined);
  const [selectedRobot, setSelectedRobot] = React.useState<any | null>(null);
  const [rosrunCommands, setRosrunCommands] = React.useState<Array<string>>([]);
  const [selectedRosrunCommand, setSelectedRosrunCommand] = React.useState<string>('');
  const [roslaunchCommands, setRoslaunchCommands] = React.useState<Array<string>>([]);
  const [selectedRoslaunchCommand, setSelectedRoslaunchCommand] = React.useState<string>('');
  const [robot, setRobot] = React.useState<any>({});
  const [connectButtonColor, setConnectButtonColor] = React.useState<any>('primary');
  const [connectButtonText, setConnectButtonText] = React.useState<any>('Connect');

  const [socket, setSocket] = React.useState<any>(null);

  useEffect(() => {
    if (robotUuids === undefined) {
      rowma.currentConnectionList().then((res: any) => {
        console.log(res.data)
        setRobotUuids(res.data.map((robot: any) => robot.uuid));
      })
    }
  });

  const classes = useStyles();

  const handleRobotChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setSelectedRobot((event.target as HTMLInputElement).value);
  };

  const handleConnectClicked = () => {
    rowma.connect(selectedRobot).then((sock: any) => {
      setSocket(sock)
    }).catch((e: any) => {
      console.log(e)
    })

    rowma.getRobotStatus(selectedRobot).then((res: any) => {
      console.log(res.data)
      setRobot(res.data)
      setRosrunCommands(res.data['rosrunCommands']);
      setRoslaunchCommands(res.data['launchCommands']);
      setConnectButtonColor('secondary');
      setConnectButtonText('Disconnect');
    })
  }

  const handleRosrunCommandChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setSelectedRosrunCommand((event.target as HTMLInputElement).value);
  };

  const handleRoslaunchCommandChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setSelectedRoslaunchCommand((event.target as HTMLInputElement).value);
  };

  const handleRosrunButtonClick = () => {
    const rosrunArgs = '';
    rowma.runRosrun(socket, selectedRobot, selectedRosrunCommand, rosrunArgs);
  }

  const handleRoslaunchButtonClick = () => {
    rowma.runLaunch(socket, selectedRobot, selectedRoslaunchCommand)
  }

  return (
    <div className={`${classes.root} App`}>
      <ThemeProvider theme={theme}>
        <AppBar position="static" className={classes.header}>
          <Toolbar>
            <Container>
              <Typography variant="h5">Rowma Network Console</Typography>
            </Container>
          </Toolbar>
        </AppBar>
        <Container>
          <Grid container spacing={3} className="py-8">
            <Grid item xs={12} sm={12} md={4}>
              <Paper className={classes.paper}>
                <div>
                  <FormControl component="fieldset" className={classes.radioButtons}>
                    <div className="my-4">
                      <Typography variant='h5'>Select Your Robot{"'"}s UUID</Typography>
                    </div>
                    {(!robotUuids || (robotUuids && robotUuids.length === 0)) &&
                      <p>Robot not found...</p>
                    }
                    <RadioGroup aria-label="robots" name="robots" value={selectedRobot} onChange={handleRobotChange} className={classes.radioGroup}>
                    {robotUuids && robotUuids.map(uuid => {
                      return (
                        <FormControlLabel value={uuid} control={<Radio />} label={uuid} />
                      )
                    })}
                    </RadioGroup>
                  </FormControl>
                </div>
                <div>
                  <Button variant="contained" color={connectButtonColor} onClick={handleConnectClicked}>
                    {connectButtonText}
                  </Button>
                </div>
              </Paper>
            </Grid>

            <Grid item xs={12} sm={12} md={4}>
              <Paper className={classes.paper}>
                <div>
                  <FormControl component="fieldset" className={classes.radioButtons}>
                    <div className="my-4">
                      <Typography variant='h5'>Select a rosrun command</Typography>
                    </div>
                    <RadioGroup aria-label="rosrun" name="rosrun" value={selectedRosrunCommand} onChange={handleRosrunCommandChange} className={classes.radioGroup}>
                    {rosrunCommands && rosrunCommands.map(command => {
                      return (
                        <FormControlLabel value={command} control={<Radio />} label={command} />
                      )
                    })}
                    </RadioGroup>
                  </FormControl>
                </div>
                <div>
                  <Button variant="contained" color="primary" onClick={handleRosrunButtonClick}>
                    Execute
                  </Button>
                </div>
              </Paper>
            </Grid>

            <Grid item xs={12} sm={12} md={4}>
              <Paper className={classes.paper}>
                <div>
                  <FormControl component="fieldset" className={classes.radioButtons}>
                    <div className="my-4">
                      <Typography variant='h5'>Select a roslaunch command</Typography>
                    </div>
                    <RadioGroup aria-label="roslaunch" name="roslaunch" value={selectedRoslaunchCommand} onChange={handleRoslaunchCommandChange} className={classes.radioGroup}>
                    {roslaunchCommands && roslaunchCommands.map(command => {
                      return (
                        <FormControlLabel value={command} control={<Radio />} label={command} />
                      )
                    })}
                    </RadioGroup>
                  </FormControl>
                </div>
                <div>
                  <Button variant="contained" color="primary" onClick={handleRoslaunchButtonClick}>
                    Execute
                  </Button>
                </div>
              </Paper>
            </Grid>

            <Grid item xs={12}>
              <Paper className={classes.paper}>
                <div>
                  <p>Send (Topic Selectbox) from (Robot Selectbox) to (Robot Selectbox)</p>
                </div>
              </Paper>
            </Grid>

            <Grid item xs={12}>
              <Paper className={classes.info}>
                <div className="my-4">
                  <Typography variant="h6">Network Information</Typography>
                </div>
                <Grid
                  container
                  direction="row"
                  justify="center"
                  alignItems="center"
                >
                  <Grid item xs={12} sm={12} md={6}>
                    <TableContainer className="pb-4">
                      <Table aria-label="simple table">
                        <TableBody>
                          <TableRow>
                            <TableCell scope="row">Network Name</TableCell>
                            <TableCell align="right">Rowma Public Network</TableCell>
                          </TableRow>
                          <TableRow>
                            <TableCell scope="row">Network Type</TableCell>
                            <TableCell align="right">Public</TableCell>
                          </TableRow>
                          <TableRow>
                            <TableCell scope="row">Network URL</TableCell>
                            <TableCell align="right">{'https://rocky-peak-54058.herokuapp.com'}</TableCell>
                          </TableRow>
                          <TableRow>
                            <TableCell scope="row">Network Location</TableCell>
                            <TableCell align="right">US</TableCell>
                          </TableRow>
                          <TableRow>
                            <TableCell scope="row">Network Owner</TableCell>
                            <TableCell align="right"><a href="https://asmsuechan.com">asmsuechan</a></TableCell>
                          </TableRow>

                        </TableBody>
                      </Table>
                    </TableContainer>
                  </Grid>
                </Grid>
              </Paper>
            </Grid>

            <Grid item xs={12}>
              <Box className={classes.footer} fontSize={16}>
                <Link className={classes.footerLink} href="">How to Use This Page</Link>
                <Link className={classes.footerLink} href="https://rowma.github.io/documentation/en/getting-started">Documentation</Link>
                <Link className={classes.footerLink} href="https://github.com/rowma/rowma">GitHub</Link>
              </Box>
            </Grid>

          </Grid>
        </Container>
      </ThemeProvider>
    </div>
  );
}

export default App
