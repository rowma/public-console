import React, { useEffect } from 'react';
import './App.css';

import { createMuiTheme, makeStyles, ThemeProvider, Theme, createStyles } from '@material-ui/core/styles';
import { green } from '@material-ui/core/colors';

import AppBar from '@material-ui/core/AppBar';
import Container from '@material-ui/core/Container';
import Toolbar from '@material-ui/core/Toolbar';
import Typography from '@material-ui/core/Typography';
import Paper from '@material-ui/core/Paper';
import Grid from '@material-ui/core/Grid';
import Box from '@material-ui/core/Box';
import Link from '@material-ui/core/Link';
import TextField from '@material-ui/core/TextField';

import Radio from '@material-ui/core/Radio';
import RadioGroup from '@material-ui/core/RadioGroup';
import FormControlLabel from '@material-ui/core/FormControlLabel';
import FormControl from '@material-ui/core/FormControl';
import Button from '@material-ui/core/Button';
import CircularProgress from '@material-ui/core/CircularProgress';

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
      main: '#38B48B',
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
    },
    textField: {
      width: '60%',
    },
    buttonProgress: {
      color: green[500],
      position: 'absolute',
      top: '50%',
      left: '50%',
      marginTop: -12,
      marginLeft: -12,
    },
  })
));

const sleep = (ms: number) => {
  return new Promise(resolve => setTimeout(resolve, ms));
}

interface NetworkInformationInterface {
  name: string;
  type: string;
  location: string;
  owner: string;
  version: string;
  url: string;
}

const emptyNetworkInformation: NetworkInformationInterface = {
  name: '',
  type: '',
  location: '',
  owner: '',
  version: '',
  url: ''
}

const App: React.FC = () => {
  const [rowmaUrl, setRowmaUrl] = React.useState<string>("https://rocky-peak-54058.herokuapp.com");
  const [rowma, setRowma] = React.useState<any>(null);
  const [robotUuids, setRobotUuids] = React.useState<Array<string> | undefined>(undefined);
  const [selectedRobot, setSelectedRobot] = React.useState<any | null>(null);
  const [rosrunCommands, setRosrunCommands] = React.useState<Array<string>>([]);
  const [selectedRosrunCommand, setSelectedRosrunCommand] = React.useState<string>('');
  const [roslaunchCommands, setRoslaunchCommands] = React.useState<Array<string>>([]);
  const [rosnodes, setRosnodes] = React.useState<Array<string>>([]);
  const [selectedRoslaunchCommand, setSelectedRoslaunchCommand] = React.useState<string>('');
  const [selectedRosnode, setSelectedRosnode] = React.useState<string>('');
  const [robot, setRobot] = React.useState<any>({});
  const [submitUrlButtonLoading, setSubmitUrlButtonLoading] = React.useState<boolean>(false);
  const [connectButtonLoading, setConnectButtonLoading] = React.useState<boolean>(false);
  const [rosrunButtonLoading, setRosrunButtonLoading] = React.useState<boolean>(false);
  const [roslaunchButtonLoading, setRoslaunchButtonLoading] = React.useState<boolean>(false);
  const [rosnodeButtonLoading, setRosnodeButtonLoading] = React.useState<boolean>(false);
  const [networkInformation, setNetworkInformation] = React.useState<any>(emptyNetworkInformation);

  const [socket, setSocket] = React.useState<any>(null);

  const classes = useStyles();

  const handleUrlFieldChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setRowmaUrl((event.target as HTMLInputElement).value);
  }

  const handleConnectNetworkClick = async () => {
    setSubmitUrlButtonLoading(true);
    const _rowma = new Rowma({ baseURL: rowmaUrl })
    setRowma(_rowma);

    const networkInfo = await _rowma.getNetworkInformation()
    setNetworkInformation({ url: rowmaUrl, ...networkInfo.data })

    const connList = await _rowma.currentConnectionList()
    setRobotUuids(connList.data.map((robot: any) => robot.uuid));

    setRobot({})
    setRosrunCommands([]);
    setRoslaunchCommands([]);

    setSubmitUrlButtonLoading(false);
  }

  const handleRobotChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setSelectedRobot((event.target as HTMLInputElement).value);
  };

  const handleConnectClicked = () => {
    setConnectButtonLoading(true);
    rowma.connect(selectedRobot).then((sock: any) => {
      setSocket(sock)
    }).catch((e: any) => {
      console.log(e)
    })

    rowma.getRobotStatus(selectedRobot).then((res: any) => {
      setRobot(res.data)
      setRosnodes(res.data.rosnodes)
      setRosrunCommands(res.data.rosrunCommands);
      setRoslaunchCommands(res.data.launchCommands);
      setConnectButtonLoading(false);
    })
  }

  const handleRosrunCommandChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setSelectedRosrunCommand((event.target as HTMLInputElement).value);
  };

  const handleRoslaunchCommandChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setSelectedRoslaunchCommand((event.target as HTMLInputElement).value);
  };

  const handleRosrunButtonClick = async () => {
    setRosrunButtonLoading(true);
    setRosnodeButtonLoading(true);
    const rosrunArgs = '';
    await rowma.runRosrun(socket, selectedRobot, selectedRosrunCommand, rosrunArgs);
    setRosrunButtonLoading(false);
    await sleep(2500);
    const _robot = await rowma.getRobotStatus(selectedRobot)
    setRosnodes(_robot.data.rosnodes)
    setRosnodeButtonLoading(false);
  }

  const handleRoslaunchButtonClick = async () => {
    setRoslaunchButtonLoading(true);
    setRosnodeButtonLoading(true);
    const result = await rowma.runLaunch(socket, selectedRobot, selectedRoslaunchCommand)
    setRoslaunchButtonLoading(false);
    await sleep(2500);
    const _robot = await rowma.getRobotStatus(selectedRobot)
    setRosnodes(_robot.data.rosnodes)
    setRosnodeButtonLoading(false);
  }

  const handleRosnodeChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setSelectedRosnode((event.target as HTMLInputElement).value);
  }

  const handleRosnodeButtonClick = async () => {
    setRosnodeButtonLoading(true);
    const result = await rowma.killNodes(socket, selectedRobot, [selectedRosnode]);
    if (result.status === 'success') {
      const index = rosnodes.indexOf(selectedRosnode)
      rosnodes.splice(index, 1);
    }
    setRosnodeButtonLoading(false);
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
            <Grid item xs={12} sm={12} md={12}>
              <Paper className={classes.paper}>
                <div className="flex items-center justify-center">
                  <TextField color="secondary" margin="dense" label="Network URL" variant="outlined" className={classes.textField} onChange={handleUrlFieldChange} value={rowmaUrl} />
                  <div className="relative mx-4">
                    <Button
                      variant="contained"
                      color="primary"
                      disabled={submitUrlButtonLoading}
                      onClick={handleConnectNetworkClick}
                    >
                      Connect
                    </Button>
                    {submitUrlButtonLoading && <CircularProgress size={24} className={classes.buttonProgress} />}
                  </div>
                </div>
              </Paper>
            </Grid>

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
                <div className="relative">
                  <Button
                    variant="contained"
                    color="primary"
                    disabled={connectButtonLoading || !selectedRobot}
                    onClick={handleConnectClicked}
                  >
                    Connect
                  </Button>
                  {connectButtonLoading && <CircularProgress size={24} className={classes.buttonProgress} />}
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
                <div className="relative">
                  <Button
                    variant="contained"
                    color="primary"
                    disabled={rosrunButtonLoading || selectedRosrunCommand === ''}
                    onClick={handleRosrunButtonClick}
                  >
                    Execute
                  </Button>
                  {rosrunButtonLoading && <CircularProgress size={24} className={classes.buttonProgress} />}
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
                <div className="relative">
                  <Button
                    variant="contained"
                    color="primary"
                    disabled={roslaunchButtonLoading || selectedRoslaunchCommand === ''}
                    onClick={handleRoslaunchButtonClick}
                  >
                    Execute
                  </Button>
                  {roslaunchButtonLoading && <CircularProgress size={24} className={classes.buttonProgress} />}
                </div>
              </Paper>
            </Grid>

            <Grid item xs={12} sm={12} md={4}>
              <Paper className={classes.paper}>
                <div>
                  <FormControl component="fieldset" className={classes.radioButtons}>
                    <div className="my-4">
                      <Typography variant='h5'>Running ROS nodes</Typography>
                    </div>
                    <RadioGroup aria-label="rosnodes" name="rosnodes" value={selectedRosnode} onChange={handleRosnodeChange} className={classes.radioGroup}>
                    {rosnodes && rosnodes.map((node: any) => {
                      return (
                        <FormControlLabel value={node} control={<Radio />} label={node} />
                      )
                    })}
                    </RadioGroup>
                  </FormControl>
                </div>
                <div className="relative">
                  <Button
                    variant="contained"
                    color="primary"
                    disabled={rosnodeButtonLoading || selectedRosnode === ''}
                    onClick={handleRosnodeButtonClick}
                  >
                    Kill
                  </Button>
                  {rosnodeButtonLoading && <CircularProgress size={24} className={classes.buttonProgress} />}
                </div>
              </Paper>
            </Grid>

            <Grid item xs={12} sm={12} md={8}>
              <Paper className={classes.paper}>
                <div>
                  <FormControl component="fieldset" className={classes.radioButtons}>
                    <div className="my-4">
                      <Typography variant='h5'>Subscribe rostopic</Typography>
                    </div>

                    <RadioGroup aria-label="rosnodes" name="rosnodes" value={selectedRosnode} onChange={handleRosnodeChange} className={classes.radioGroup}>
                    </RadioGroup>
                  </FormControl>
                </div>
                <div className="relative">
                  <Button
                    variant="contained"
                    color="primary"
                    onClick={() => {}}
                  >
                    Subscribe
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
                            <TableCell align="right">{networkInformation.name}</TableCell>
                          </TableRow>
                          <TableRow>
                            <TableCell scope="row">Network Type</TableCell>
                            <TableCell align="right">{networkInformation.type}</TableCell>
                          </TableRow>
                          <TableRow>
                            <TableCell scope="row">Network URL</TableCell>
                            <TableCell align="right">{networkInformation.url}</TableCell>
                          </TableRow>
                          <TableRow>
                            <TableCell scope="row">Network Location</TableCell>
                            <TableCell align="right">{networkInformation.location}</TableCell>
                          </TableRow>
                          <TableRow>
                            <TableCell scope="row">Network Owner</TableCell>
                            <TableCell align="right">{networkInformation.owner}</TableCell>
                          </TableRow>
                          <TableRow>
                            <TableCell scope="row">Network Version</TableCell>
                            <TableCell align="right">{networkInformation.version}</TableCell>
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
