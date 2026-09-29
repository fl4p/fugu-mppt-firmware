import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

const sections = [
  {title: 'Guide', to: '/docs/guide/intro', text: 'Hardware, build, flash, provision, connect, update and charge.'},
  {title: 'Reference', to: '/docs/reference/config', text: 'Configuration files, console commands, MQTT topics, telemetry fields.'},
  {title: 'How it works', to: '/docs/internals/architecture', text: 'Control loop, MPPT, sensors, filters, diode emulation, PWM drivers.'},
  {title: 'Lab', to: '/docs/lab', text: 'Bench setups, power-loop rig, measurements and automated tests.'},
  {title: 'Development', to: '/docs/development/repo-layout', text: 'Repo layout, build, conventions, testing and debugging.'},
];

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout description={siteConfig.tagline}>
      <header className={styles.hero}>
        <div className="container">
          <h1>{siteConfig.title}</h1>
          <p className={styles.tagline}>{siteConfig.tagline}</p>
          <Link className="button button--primary button--lg" to="/docs/guide/getting-started">
            Get started
          </Link>
        </div>
      </header>
      <main className="container margin-vert--xl">
        <div className={styles.grid}>
          {sections.map((s) => (
            <Link key={s.title} to={s.to} className={styles.card}>
              <h3>{s.title}</h3>
              <p>{s.text}</p>
            </Link>
          ))}
        </div>
      </main>
    </Layout>
  );
}
