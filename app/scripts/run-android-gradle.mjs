import { existsSync, readdirSync } from 'node:fs';
import { spawnSync } from 'node:child_process';
import path from 'node:path';
import process from 'node:process';
import { fileURLToPath } from 'node:url';

const appRoot = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const androidRoot = path.join(appRoot, 'android');
const gradleCommand = process.platform === 'win32' ? 'gradlew.bat' : './gradlew';
const requestedTasks = process.argv.slice(2);

if (requestedTasks.length === 0) {
  throw new Error('Pass at least one Gradle task, for example assembleDebug');
}

function javaMajor(javaHome) {
  const java = path.join(javaHome, 'bin', process.platform === 'win32' ? 'java.exe' : 'java');
  if (!existsSync(java)) return null;
  const result = spawnSync(java, ['-version'], { encoding: 'utf8' });
  const match = `${result.stdout}${result.stderr}`.match(/version "(\d+)/);
  return match ? Number(match[1]) : null;
}

function isFullJdk21(javaHome) {
  const executableSuffix = process.platform === 'win32' ? '.exe' : '';
  return (
    javaMajor(javaHome) === 21 &&
    existsSync(path.join(javaHome, 'bin', `javac${executableSuffix}`)) &&
    existsSync(path.join(javaHome, 'bin', `jlink${executableSuffix}`))
  );
}

function javaCandidates() {
  const candidates = [process.env.DRONTECT_ANDROID_JAVA_HOME, process.env.JAVA_HOME];
  if (process.platform === 'win32') {
    const javaRoot = path.join(process.env.ProgramFiles || 'C:\\Program Files', 'Java');
    candidates.push(path.join(javaRoot, 'jdk-21'));
    if (existsSync(javaRoot)) {
      const installed = readdirSync(javaRoot, { withFileTypes: true })
        .filter((entry) => entry.isDirectory())
        .sort((left, right) => left.name.localeCompare(right.name))
        .map((entry) => path.join(javaRoot, entry.name));
      candidates.push(...installed);
    }
    candidates.push(path.join(process.env.ProgramFiles || 'C:\\Program Files', 'Android', 'Android Studio', 'jbr'));
  }
  return [...new Set(candidates.filter(Boolean))];
}

function sdkCandidates() {
  const candidates = [
    process.env.DRONTECT_ANDROID_SDK_ROOT,
    process.env.ANDROID_HOME,
    process.env.ANDROID_SDK_ROOT,
  ];
  if (process.platform === 'win32' && process.env.LOCALAPPDATA) {
    candidates.push(path.join(process.env.LOCALAPPDATA, 'Android', 'Sdk'));
  }
  return [...new Set(candidates.filter(Boolean))];
}

const javaHome = javaCandidates().find(isFullJdk21);
if (!javaHome) {
  throw new Error(
    'Android builds require a full JDK 21 with javac and jlink. ' +
      'Set DRONTECT_ANDROID_JAVA_HOME to your JDK 21 installation.',
  );
}

const sdkRoot = sdkCandidates().find((candidate) =>
  existsSync(path.join(candidate, 'platforms', 'android-35', 'android.jar')),
);
if (!sdkRoot) {
  throw new Error(
    'Android SDK platform 35 was not found. Set DRONTECT_ANDROID_SDK_ROOT after installing it.',
  );
}

console.info(`Android build using JDK ${javaMajor(javaHome)} at ${javaHome}`);
console.info(`Android build using SDK at ${sdkRoot}`);

function gradleSystemProperty(name, value) {
  const argument = `-D${name}=${value}`;
  return process.platform === 'win32' ? `"${argument}"` : argument;
}

// Keep the Android SDK/JDK environment isolated to this invocation. In
// particular, do not reuse a daemon first started by a shell whose JAVA_HOME
// points at an unsupported JDK and whose Android SDK variables may differ.
const gradleArguments = [
  ...requestedTasks,
  '--no-daemon',
  gradleSystemProperty('org.gradle.java.home', javaHome),
  gradleSystemProperty('org.gradle.java.installations.paths', javaHome),
  '-Dorg.gradle.java.installations.auto-detect=false',
  '-Dorg.gradle.java.installations.auto-download=false',
];

const result = spawnSync(gradleCommand, gradleArguments, {
  cwd: androidRoot,
  env: {
    ...process.env,
    JAVA_HOME: javaHome,
    ANDROID_HOME: sdkRoot,
    ANDROID_SDK_ROOT: sdkRoot,
  },
  stdio: 'inherit',
  shell: process.platform === 'win32',
});

if (result.error) throw result.error;
process.exit(result.status ?? 1);
